#include "sensors.h"
#include "board.h"

#include <string.h>
#include "Drivers/SPI/spiDriver.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lcdInterface.h"
#include "Drivers/PWM/pwmDriver.h"
#include "adc.h"

/// \file sensors.c All Sensor information and registers
/** \file
 * This file describes all the sensors and interfaces using
 * structures defined in sensors.h. This is where you would add
 * a new sensor, or a new thermal control device.
 */

/******************************
 * Interface and reg setup    *
 ******************************/

//PWM Controller
#define PID_KP 20 //orig = 50
#define PID_KI 3 //orig = 5
#define PID_KD 0

 
// ADT7473
#define ADT7473_ID		0x73
#define ADT7473_LIMIT_N (sizeof(adt_limit)/sizeof(int))
static int adt_limit[] = { 0x4F, 0x51 };

// ADT7461
#define ADT7461_ID 		0x41

// MAX6646, MAX6647, MAX6649
#define TEMP_LIMIT_N (sizeof(wr_temp_limit_max)/sizeof(int))
static int wr_temp_limit_max[] = { 0x0D, 0x0B, 0x19, 0x20 };

// NVIDIA internal sensor
#define NVIDIA8_ID		0x00
#define NVIDIA9_ID		0xDE
#define NVIDIA8_LIMIT_N (sizeof(nvidia8_limit)/sizeof(int))
static int nvidia8_limit[] = { 0x20, 0x05 };

#define REGTABLE_N (sizeof(regTable)/sizeof(t_regtable))
/// registry table for the i2c devices
static const t_regTable regTable[] = {
    { 0x01, 0x10, TEMP_LIMIT_N,    wr_temp_limit_max, 0x07, 0x02, 0xFE },
    { 0x25, 0x77, ADT7473_LIMIT_N, adt_limit,         0x4F, 0,    0x3D },
    { 0x00, 0x11, NVIDIA8_LIMIT_N, nvidia8_limit,     0x20, 0,    0xFE },
};
int generic_i2c_data_process(sensor *sense, int low, int high);
int adt7473_i2c_data_process(sensor *sense, int low, int high);

///process data from i2c devices
/**
	Convert the low and high values into a temp value
	\param sense the sensor in use
	\param low the low byte (or bytes)
	\param high the high byte (or bytes)
	\return tempature (as a signed value)
*/
int generic_i2c_data_process(sensor *sense, int low, int high) {
	high -= sense->priv_i2c->i2c->offset;
	return (100 * high) + ((100 * low) / 256);
}

///process data from adt7473
/**
	Convert the low and high values into a temp value, this device does it differently
	than all the other i2c devices.
	\param sense the sensor in use
	\param low the low byte (or bytes)
	\param high the high byte (or bytes)
	\return tempature (as a signed value)
*/
int adt7473_i2c_data_process(sensor *sense, int low, int high) {
	low &= 0x0C;
	low <<= 4;
	return generic_i2c_data_process(sense, low, high);
}

#define I2C_NV99		0
#define I2C_NV9 		1
#define I2C_NV8 		2
#define I2C_ADT7461e 	3
#define I2C_6649		4
#define I2C_ADT7473		5
#define I2C_LM99		6
#define I2C_ADT7461d	7
#define I2C_6646		8
#define I2C_TMP411A		9
#define I2C_TMP411B		10
///table of all the possible i2c devices
	// convTime,offset,addr,deviceId,minThres,maxThres,text,pRegTable,process_data
const i2c_info i2c_devices[] = {
    { 50,  0,   0x4F, NVIDIA9_ID, 0,   127,  "NVIDIA9 - 0x4F", &(regTable[2]), generic_i2c_data_process},
    { 200,  0,   0x9e, 0x9e,       0,   127,  "LM75 - 0x9E", &(regTable[2]), generic_i2c_data_process},
    { 50,  0,   0x4F, NVIDIA8_ID, 0,   127,  "NVIDIA8 - 0x4F", &(regTable[2]), generic_i2c_data_process},
    { 200, 0,   0x4C, ADT7461_ID, 0,   127,  "ADT7461 - 0x4C", &(regTable[0]), generic_i2c_data_process},
    { 250, 0,   0x4C, 0x4D,       0,   127,  "Unknown - 0x4D", &(regTable[0]), generic_i2c_data_process},
    { 63,  64,  0x2E, ADT7473_ID, -63, 191,  "ADT7473 - 0x2E", &(regTable[1]), adt7473_i2c_data_process},
    { 69,  -16, 0x4C, 0x01,       16,  140,  "Unknown - 0x01", &(regTable[0]), generic_i2c_data_process},
/*#ifdef G80_ATE
    { 50,  64,  0x4D, ADT7461_ID, -55, 150,  "7461-e", &(regTable[0]) },
#else*/
    { 200, 0,   0x4D, ADT7461_ID, 0,   127,  "ADT7461 - 0x4D", &(regTable[0]), generic_i2c_data_process},
    { 250, 0,   0x4D, 0x4D,       0,   127,  "Unknown - 0x4D", &(regTable[0]), generic_i2c_data_process},
     { 200, 0,   0x4C, 0x55,       0,   127,  "TMP411A - 0x4C", &(regTable[0]), generic_i2c_data_process},
    { 200, 0,   0x4D, 0x55,       0,   127,  "TMP411B - 0x4D", &(regTable[0]), generic_i2c_data_process},
};

#define SPI_N (sizeof(spi_devices)/sizeof(spi_info))
void spi_update_temp1(spi_transfer *trans);
void spi_update_temp2(spi_transfer *trans);

///table of the spi devices
static spi_info spi_devices[] = {
    {0, {0, 0}, {0xffff, 0xffff}, spi_update_temp1},
    {2, {0, 0}, {0xffff, 0xffff}, spi_update_temp2}
};

/***********************************
 * Setup sensors, sensor functions *
 ***********************************/
unsigned int i2c_sensor_update(sensor *sense);

///update an i2c sensor
/**
	Updates an i2c sensor, this function blocks until operation completes.
	Note that an error can be returned if the device isn't present, or returns
	an invaid temp.

	\param sense sensor to update
	\return non zero on error
*/
unsigned int i2c_sensor_update(sensor *sense) {
	int high = 0;
	int low = 0;
	int status = 0;
	i2c_info *p_i2c = sense->priv_i2c->i2c;
	//first we read the status register, if it exists
	if (p_i2c->pRegTable->reg_rstatus) {
		//we have a status register
		if (!I2CReadThermalDeviceRegister(sense->priv_i2c->port, p_i2c->addr, p_i2c->pRegTable->reg_rstatus, &status))
			return 1;
		if (status & 0x04) //error
			return 2;
	}
	//next we read the LSB
	if (!I2CReadThermalDeviceRegister(sense->priv_i2c->port, p_i2c->addr, p_i2c->pRegTable->reg_rtempl, &low))
		return 3;
	
	//finally the MSB
	if (!I2CReadThermalDeviceRegister(sense->priv_i2c->port, p_i2c->addr, p_i2c->pRegTable->reg_rtemph, &high))
		return 4;

	sense->value = p_i2c->process_data(sense, low, high);
	sense->time = xTaskGetTickCount();
	//sanity check the value
	if ((sense->value >= (p_i2c->maxThres * 100))||(sense->value <= (p_i2c->minThres * 100)))
		return 5;

	//trigger redraw
	LCD_info_changed();
	
	return 0;
}

//Function prototypes
void spi_update_temp(unsigned short);
signed int linearize_tcouple(signed int cj_temp,signed int tc_temp,int tc_type);

///update thermocouple 1 temp
void spi_update_temp1(spi_transfer *trans) {
    spi_update_temp(THERMOCOUPLE_1);
}

///update thermocouple 2 temp
void spi_update_temp2(spi_transfer *trans) {
    spi_update_temp(THERMOCOUPLE_2);
}	

unsigned int spi_update(sensor *sense);

///update an spi device, uses the blocking spi call
unsigned int spi_update(sensor *sense) {
    vSPIDoTransfer(sense->priv_spi->spi_cs, sense->priv_spi->tx, sense->priv_spi->rx, 2, sense->priv_spi->call);
    return 0;
}

///i2c table of port + device pointers
static const i2c_full_info i2c_privs[] = {
	{I2C_PORT1, &i2c_devices[I2C_NV99]},
	{I2C_PORT2, &i2c_devices[I2C_NV99]},
	{I2C_PORT1, &i2c_devices[I2C_NV9]},
	{I2C_PORT2, &i2c_devices[I2C_NV9]},
	{I2C_PORT1, &i2c_devices[I2C_NV8]},
	{I2C_PORT2, &i2c_devices[I2C_NV8]},
	{I2C_PORT1, &i2c_devices[I2C_ADT7461e]},
	{I2C_PORT2, &i2c_devices[I2C_ADT7461e]},
	{I2C_PORT1, &i2c_devices[I2C_6649]},
	{I2C_PORT2, &i2c_devices[I2C_6649]},
	{I2C_PORT1, &i2c_devices[I2C_ADT7473]},
	{I2C_PORT2, &i2c_devices[I2C_ADT7473]},
	{I2C_PORT1, &i2c_devices[I2C_LM99]},
	{I2C_PORT2, &i2c_devices[I2C_LM99]},
	{I2C_PORT1, &i2c_devices[I2C_ADT7461d]},
	{I2C_PORT2, &i2c_devices[I2C_ADT7461d]},
	{I2C_PORT1, &i2c_devices[I2C_6646]},
	{I2C_PORT2, &i2c_devices[I2C_6646]},
	{I2C_PORT1, &i2c_devices[I2C_TMP411A]},
	{I2C_PORT2, &i2c_devices[I2C_TMP411A]},
	{I2C_PORT1, &i2c_devices[I2C_TMP411B]},
	{I2C_PORT2, &i2c_devices[I2C_TMP411B]},
};

///checks if an i2c sensor is connected
unsigned char i2cConnected(sensor *s) {
	return I2CCheckSensor(s->priv_i2c->port, s->priv_i2c->i2c->addr, s->priv_i2c->i2c->pRegTable->reg_rid, s->priv_i2c->i2c->deviceId);
}

///checks if a thermocouple sensor is connected (always is)
unsigned char thermoCoupleConnected(sensor *s) {
	return 1;
}

///table of sensors, includes sensor priority, device tables, callbacks, etc
sensor sensors[] = {
    {"Thermocouple1", 			"TCPL1", 	0, 	0, 	{.priv_spi = &spi_devices[0]}, 	spi_update, 		1,	thermoCoupleConnected},
    {"Thermocouple2", 			"TCPL2", 	0, 	0, 	{.priv_spi = &spi_devices[1]}, 	spi_update, 		1,	thermoCoupleConnected},
    {"NV9x, NV10x bus 1",		"NV9/10",	0,	0,	{.priv_i2c = &i2c_privs[0]}, 	i2c_sensor_update,	3,	i2cConnected},
    {"NV9x, NV10x bus 2",		"NV9/10",	0,	0,	{.priv_i2c = &i2c_privs[1]}, 	i2c_sensor_update,	3,	i2cConnected},
    {"NV9x, NV10x bus 1",		"NV9/10",	0,	0,	{.priv_i2c = &i2c_privs[2]}, 	i2c_sensor_update,	3,	i2cConnected},
    {"NV9x, NV10x bus 2",		"NV9/10",	0,	0,	{.priv_i2c = &i2c_privs[3]}, 	i2c_sensor_update,	3,	i2cConnected},
    {"NV8x bus 1",				"NV8-i ",	0,	0,	{.priv_i2c = &i2c_privs[4]}, 	i2c_sensor_update,	3,	i2cConnected},
    {"NV8x bus 2",				"NV8-i ",	0,	0,	{.priv_i2c = &i2c_privs[5]}, 	i2c_sensor_update,	3,	i2cConnected},
    {"ADT7461e bus 1",			"7461-e",	0,	0,	{.priv_i2c = &i2c_privs[6]}, 	i2c_sensor_update,	2,	i2cConnected},
    {"ADT7461e bus 2",			"7461-e",	0,	0,	{.priv_i2c = &i2c_privs[7]}, 	i2c_sensor_update,	2,	i2cConnected},
    {"6649 bus 1",				"6649-e",	0,	0,	{.priv_i2c = &i2c_privs[8]}, 	i2c_sensor_update,	2,	i2cConnected},
    {"6649 bus 2",				"6649-e",	0,	0,	{.priv_i2c = &i2c_privs[9]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"ADT7473 bus 1",			"7473-e",	0,	0,	{.priv_i2c = &i2c_privs[10]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"ADT7473 bus 2",			"7473-e",	0,	0,	{.priv_i2c = &i2c_privs[11]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"LM99 bus 1",				"LM99-e",	0,	0,	{.priv_i2c = &i2c_privs[12]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"LM99 bus 2",				"LM99-e",	0,	0,	{.priv_i2c = &i2c_privs[13]}, 	i2c_sensor_update,	2,	i2cConnected},
    {"ADT7461d bus 1",			"7461-d",	0,	0,	{.priv_i2c = &i2c_privs[14]}, 	i2c_sensor_update,	2,	i2cConnected},
    {"ADT7461d bus 2",			"7461-d",	0,	0,	{.priv_i2c = &i2c_privs[15]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"6646 bus 1",				"6646-d",	0,	0,	{.priv_i2c = &i2c_privs[16]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"6646 bus 2",				"6646-d",	0,	0,	{.priv_i2c = &i2c_privs[17]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"TMP411A bus 1",			"TMP411",	0,	0,	{.priv_i2c = &i2c_privs[18]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"TMP411A bus 2",			"TMP411",	0,	0,	{.priv_i2c = &i2c_privs[19]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"TMP411B bus 1",			"TMP411",	0,	0,	{.priv_i2c = &i2c_privs[20]}, 	i2c_sensor_update,	2,	i2cConnected},
	{"TMP411B bus 2",			"TMP411",	0,	0,	{.priv_i2c = &i2c_privs[21]}, 	i2c_sensor_update,	2,	i2cConnected},
};

//updates the temp on an spi device (this is a callback from an spi transfer function)
void spi_update_temp(unsigned short num) {
    //perform thermocouple conversion, recall that value is temp*100
	unsigned int v = sensors[num].priv_spi->rx[0];
	unsigned int s = sensors[num].priv_spi->rx[1];

	double cj_temp;// cold junction temperature
	double tc_temp;// thermocouple temperature
	unsigned char type = (v == s) ? 1 : 0; //1 for K, 0 for T
	if (type) {
		sensors[num].value = ((v * 100)>>5);
	}
	else {
		if (s & 1)
			sensors[num].value = SENSOR_OPEN;
		else{
			cj_temp = (double)((signed short int)s>>4)*6.25;
			tc_temp = (double)(((signed short int)v) >> 2)*25;
			sensors[num].value = linearize_tcouple(cj_temp,tc_temp,0);
		}
	}
    sensors[num].time = xTaskGetTickCount();
    LCD_info_changed();
}
//Use a look-up table to determine linear approximation error and subtract it from the temp
//This is necessary for T-Type thermocouples as the linear approximation of the ADC can be quite off for our temperature range
signed int linearize_tcouple(signed int cj_temp,signed int tc_temp,int tc_type){
	static const signed int tc_error_lookup[256]={1692,1656,1620,1584,1549,1514,1479,1444,1410,1375,1341,1307,1273,1240,1206,1173,1140,1107,1075,1042,1010,978,946,915,883,852,821,790,759,729,698,668,638,609,579,550,520,491,463,434,405,377,349,321,293,266,239,211,184,157,131,104,78,52,26,0,-26,-51,-77,-102,-127,-152,-177,-202,-226,-251,-275,-299,-323,-347,-371,-394,-418,-441,-464,-487,-510,-532,-555,-577,-599,-621,-643,-664,-686,-707,-728,-749,-770,-791,-811,-831,-852,-872,-891,-911,-931,-950,-969,-988,-1007,-1026,-1044,-1062,-1081,-1099,-1117,-1134,-1152,-1169,-1186,-1203,-1220,-1237,-1254,-1270,-1286,-1302,-1318,-1334,-1350,-1365,-1380,-1395,-1410,-1425,-1440,-1454,-1469,-1483,-1497,-1511,-1525,-1538,-1552,-1565,-1578,-1591,-1604,-1617,-1629,-1642,-1654,-1666,-1678,-1690,-1702,-1713,-1725,-1736,-1747,-1758,-1769,-1779,-1790,-1800,-1811,-1821,-1831,-1841,-1850,-1860,-1869,-1879,-1888,-1897,-1906,-1915,-1923,-1932,-1940,-1948,-1956,-1964,-1972,-1980,-1988,-1995,-2002,-2009,-2017,-2023,-2030,-2037,-2043,-2050,-2056,-2062,-2068,-2074,-2080,-2086,-2091,-2097,-2102,-2107,-2112,-2117,-2122,-2127,-2131,-2136,-2140,-2144,-2148,-2152,-2156,-2159,-2163,-2167,-2170,-2173,-2176,-2179,-2182,-2185,-2187,-2190,-2192,-2195,-2197,-2199,-2201,-2202,-2204,-2206,-2207,-2208,-2210,-2211,-2212,-2213,-2213,-2214,-2215,-2215,-2215,-2215,-2216,-2216,-2215,-2215,-2215,-2214,-2214,-2213,-2212,-2211,-2210,-2209,-2208,-2206,-2205,-2203,-2202,-2200};
	const signed int offset =-100; //This can be used to correct for static offset, keep in mind temp *100
	int lookup_val = (int)((tc_temp-cj_temp)/100)+55;
	//Make sure value is within the table otherwise clip to min or max
	if (lookup_val <0)
		lookup_val = 0;
	if (lookup_val >255)
		lookup_val =255;
	return tc_temp - tc_error_lookup[lookup_val] + offset;
}
/***********************************
 * Setup drivers, driver functions *
 ***********************************/
///pin direction for hbridge 1
#define HBRIDGE_DIR_1 (1 << 26)
///pin direction for hbridge 2
#define HBRIDGE_DIR_2 (1 << 24)

///set hbridge 1 to heat direction
#define HBRIDGE_1_HEAT (AT91C_BASE_PIOA->PIO_SODR = HBRIDGE_DIR_1)
///set hbridge 1 to cool direction
#define HBRIDGE_1_COOL (AT91C_BASE_PIOA->PIO_CODR = HBRIDGE_DIR_1)
///set hbridge 2 to heat direction
#define HBRIDGE_2_HEAT (AT91C_BASE_PIOA->PIO_SODR = HBRIDGE_DIR_2)
///set hbridge 2 to cool direction
#define HBRIDGE_2_COOL (AT91C_BASE_PIOA->PIO_CODR = HBRIDGE_DIR_2)

//#define FAN_1_COUNT (AT91C_BASE_PIOA->PIO_SODR = FAN1_SPD)
//#define FAN_2_COUNT (AT91C_BASE_PIOA->PIO_SODR = FAN2_SPD)


///set hbridge x to heat
#define HBRIDGE_HEAT(x) \
	if (x == 1) \
		HBRIDGE_1_HEAT; \
	else \
		HBRIDGE_2_HEAT;

///set hbridge x to cool
#define HBRIDGE_COOL(x) \
	if (x == 1) \
		HBRIDGE_1_COOL; \
	else \
		HBRIDGE_2_COOL;

void turnFanOn(driver *fan, unsigned short percent);
void turnFanOff(driver *fan);
void heaterOn(driver *heater, unsigned short percent);
void peltCool(driver *pelt, unsigned short percent);
void peltHeat(driver *pelt, unsigned short percent);
void heaterOff(driver *heater);
void ctrlFanPwm(driver *fan, unsigned short percent);
///turns on the fan (100%)
void turnFanOn(driver *fan, unsigned short percent) {
	static unsigned int v;
	//need to check the current voltage
	ADC_StartConversion(AT91C_BASE_ADC);
	//wait for conversion to finish
	while (!(ADC_GetStatus(AT91C_BASE_ADC) & AT91C_ADC_EOC4));
	v = ADC_GetLastConvertedData(AT91C_BASE_ADC);
	//in hardware there is a ~1:10 divider, so 1.2 = 12v and 2.4=24v
	//we want to ramp so that 12v = 100% and 24V = 50%, we have a 3.3v ref
	//So 12v = 1.2/3.3*1024=372; 24v = 745, this gives:
	//d = 100 - (50*(v-372))/(745-372)
	if (v <= 372)
	    set_fan(fan->channel, DUTY_CYCLE_FAN(100));
	else if (v >= 745)
		set_fan(fan->channel, DUTY_CYCLE_FAN(50));
	else
		set_fan(fan->channel, DUTY_CYCLE_FAN(100 - (50*(v-372))/(745-372)));
    //if ~24V
    fan->mode = DRIVER_MODE_COOL;
   
}
void ctrlFanPwm(driver *fan, unsigned short percent)
{
	static unsigned int v;
	//need to check the current voltage
	unsigned int base = 100;
	unsigned short x = 0;
	ADC_StartConversion(AT91C_BASE_ADC);
	//wait for conversion to finish
	while (!(ADC_GetStatus(AT91C_BASE_ADC) & AT91C_ADC_EOC4));
	v = ADC_GetLastConvertedData(AT91C_BASE_ADC);
	if (v <= 372)
	  base = 100;
	else if (v >= 745)
	  base = 50;
	else
	  base = 100 - (50*(v-372))/(745-372);

	x = base * percent / 100;

	set_fan(fan->channel, DUTY_CYCLE_FAN(x));
	fan->mode = DRIVER_MODE_COOL;
}
///turns of the fan (0%)
void turnFanOff(driver *fan) {
    set_fan(fan->channel, DUTY_CYCLE_FAN(0));
    fan->mode = DRIVER_MODE_OFF;
}

///turns on the heater (100%)
void heaterOn(driver *heater, unsigned short percent) {
	if (!(heater->mode == DRIVER_MODE_HEAT))
		set_heat(heater->channel, DUTY_CYCLE_HEAT(0)); //Make sure we disable H-Bridge before we change directions
	HBRIDGE_HEAT(heater->channel);
    set_heat(heater->channel, DUTY_CYCLE_HEAT(percent));
    heater->mode = DRIVER_MODE_HEAT;
   
}

///turns the pelt to heat mode (100%)
void peltHeat(driver *pelt, unsigned short percent) {
	if (!(pelt->mode == DRIVER_MODE_HEAT))
		set_heat(pelt->channel, DUTY_CYCLE_HEAT(0)); //Make sure we disable H-Bridge before we change directions
	HBRIDGE_HEAT(pelt->channel);
    set_heat(pelt->channel, DUTY_CYCLE_HEAT(percent));
    pelt->mode = DRIVER_MODE_HEAT;
 
}

///turns the pelt to cool mode (100%)
void peltCool(driver *pelt, unsigned short percent) {
	if (!(pelt->mode == DRIVER_MODE_COOL))
		set_heat(pelt->channel, DUTY_CYCLE_HEAT(0)); //Make sure we disable H-Bridge before we change directions
    HBRIDGE_COOL(pelt->channel);
    set_heat(pelt->channel, DUTY_CYCLE_HEAT(percent));
    pelt->mode = DRIVER_MODE_COOL;
}

///turns off the heater (0%)
void heaterOff(driver *heater) {
    set_heat(heater->channel, DUTY_CYCLE_HEAT(0));
    heater->mode = DRIVER_MODE_OFF;
}

#define DRIVER_N (sizeof(drivers)/sizeof(driver))
///table of possible drivers
 driver drivers[] = {
 	// name,	mode,			channel,heaton,		coldon,	turnoff 				
    {"Fan1", 	DRIVER_MODE_OFF, 	1, 	0, 			turnFanOn, 	turnFanOff},
    {"Fan2", 	DRIVER_MODE_OFF, 	2, 	0, 			turnFanOn, 	turnFanOff},
    {"Heater1", DRIVER_MODE_OFF, 	1, 	heaterOn, 	0, 			heaterOff},
    {"Heater2", DRIVER_MODE_OFF, 	2, 	heaterOn, 	0, 			heaterOff},
    {"Pelt1", 	DRIVER_MODE_OFF, 	1, 	peltHeat, 	peltCool, 	heaterOff},
    {"Pelt2", 	DRIVER_MODE_OFF, 	2, 	peltHeat, 	peltCool, 	heaterOff},
//    {"Ramp1", 	DRIVER_MODE_OFF, 	1, 	peltHeat, 	peltCool, 	heaterOff},               //add
//    {"Ramp2", 	DRIVER_MODE_OFF, 	2, 	peltHeat, 	peltCool, 	heaterOff},                //add
	{"FanPwm1",  DRIVER_MODE_OFF,   1,  0,			ctrlFanPwm, turnFanOff},	// 
	{"FanPwm2",  DRIVER_MODE_OFF,   2,  0,			ctrlFanPwm, turnFanOff},	//
};

///function to set the sensor (sensor table must be defined as static)
void set_sensor(group *g, int i) {
	g->sens = &sensors[i];
}
///set the fan driver mode
 unsigned int set_fan_driver(group *g, unsigned int i)
{
    g->fan = &drivers[i];
}

///error number -> text map
const char *sensor_error_names[] = {
	(const char *)"No Err",
	(const char *)"Read Err",
	(const char *)"Timeout Err",
	(const char *)"Slope(safety)Err",
	(const char *)"Threshold Err",
	(const char *)"NC Err"
};

///on error turn off heater, turn on fan
extern void sensor_error(group *g, unsigned int errno) {
	g->last_error = errno;
	g->heater->turnoff(g->heater);
	g->fan->coldon(g->heater, 100);
	set_group_mode(g, MODE_FANON); //set to fanon so it isn't turned back on immediately
	LED_ERROR_ON;

	if (errno == SENSOR_READ_ERROR) {
		//causes a big problem with the i2c bus
		//reset the bus
		AT91C_BASE_TWI->TWI_CR |= (1 << 7); //sw reset the I2c
		Delay100NCycles(100);
		I2CInitialize();
	}
}

/*
	Define the actual groups to be used
 */
///group 1 information
 group group1 = {
    "Group 1",
    &sensors[THERMOCOUPLE_1],
    2500,
    13000,
    MODE_FANON,// MODE_IDLE,	// chenglei 19 Feb 2016: change  MODE_IDLE to MODE_FANON 
    7000,
    0,
    2000,
    10000,
    100,	// temp step, default 1
    0,
    100,
    &drivers[PELT_1],
    &drivers[FAN_1],
    0,
    PID_KP,
    PID_KI,
    PID_KD,
    0,
	//1, //0=manual, 1=225W, 2=350W
	 0,  // 0=225WS, 1=350WS
	0 ,
	0,     /// fan speed
	240,     // threshold  fan speed
	0,       // last_pid_pwm
	0,      // 0 is default pid setting by firmware , 1 is user defined pid 
	1.0,
	0.0,
};

///group 2 information
group group2 = {
    "Group 2",
    &sensors[THERMOCOUPLE_2],
    2500,
    13000,
    MODE_FANON,// MODE_IDLE,	// chenglei 19 Feb 2016: change  MODE_IDLE to MODE_FANON 
    7000,
    0,
    2000,
    10000,
    100,	// temp step, default 1
    0,
    100,
    &drivers[PELT_2],
    &drivers[FAN_2],
    0,
    PID_KP,
    PID_KI,
    PID_KD,
    0,
	//1, //0=manual, 1=225W, 2=350W
	  0,  // 0=225WS, 1=350WS
	0 ,
	0,     /// fan speed
	240,     // threshold  fan speed
	0,       // last_pid_pwm
	0,      // 0 is default pid setting by firmware , 1 is user defined pid 
	1.0,
	0.0,
};

///Selects best sensor that is available
/**
	scans all the sensor and selects the highest priority one
	that is available

	\param g sensor group to check
*/
void perform_group_scan_update(group *g) {
	unsigned char bus = ((g == S_GROUP1) ? 0 : 1);
	int i;
	unsigned char currentPriority = 0;
	unsigned char sBus = 0;
	unsigned int  sIndex = 0;
	for (i = 0; i < SENSORS_N; i++) {
		//check if we are on a proper bus, note that the spi_cs and i2c portid are aligned in the union, so we can use either
		sBus = ((sensors[i].priv_spi->spi_cs == 0) ? 0 : 1);
		if (sBus != bus)
			continue;
		sensors[i].update(&sensors[i]);
		if ((sensors[i].connected(&sensors[i]))&&(sensors[i].priority > currentPriority)&&(!sensors[i].update(&sensors[i]))) {
			currentPriority = sensors[i].priority;
			g->sens = &sensors[i];
			sIndex = i;
		}
	}
#if 1
	sensor *sense = &sensors[sIndex];
	i2c_info *p_i2c = sense->priv_i2c->i2c;
	//first we read the status register, if it exists
	unsigned int ret;
	int val = 0xFF;
	static char *Newsensorshortname = "NCT72";


	if (strcmp(sense->shortname,"7461-e")==0 || strcmp(sense->shortname,"7461-d")==0)
	{
		if (p_i2c->pRegTable->reg_rstatus) {
			//we have a status register
			if (!I2CReadThermalDeviceRegister(sense->priv_i2c->port, p_i2c->addr, 0xFF, &val))
			{
				if (val == 0x51){
				/*g->sens->shortname = Oldsensor; */
				}

			}else{
				g->sens->shortname = Newsensorshortname; 
				g->sens->name =  Newsensorshortname; 
			}
		}
	}
#endif

}



