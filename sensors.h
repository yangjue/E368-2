#ifndef SENSORS_INCLUSION_GUARD
#define SENSORS_INCLUSION_GUARD
/// \file sensors.h header to sensors.c


#include "FreeRTOS.h"
#include "task.h"

#include "Drivers/I2C/i2cDriver.h"
#include "Drivers/SPI/spiDriver.h"


//Group modes
///AUTO mode, fan and heater attempt to force temp to target
#define MODE_AUTO 0 
///FANON mode, turns the fan on, heater off
#define MODE_FANON 1
///IDLE mode, fan off, heater off
#define MODE_IDLE 2
///PELT mode, fan and pelt are used to attempt to force temp to target
#define MODE_PELT 3
///RAMP mode, ramp up and down between two temperatures
#define MODE_RAMP 4
///RAMP atuo mode for Qual cooler.
#define MODE_RAMP_QC 5   // Chenglei add at 7/17/2015. Auto ramp mode for qual cooler
#define MODE_PWM_QC 6
#define MODE_N 7    

//defines for refering to the group
///macro for pointer to group 1
#define S_GROUP1 &group1
///macro for pointer to group 2
#define S_GROUP2 &group2

//driver numbers
#define FAN_1 0
#define FAN_2 1
#define HEAT_1 2
#define HEAT_2 3
#define PELT_1 4
#define PELT_2 5
//#define RAMP_1 6
//#define RAMP_2 7
#define FANPWM1 6
#define FANPWM2 7

#define INT_WIN_SIZE 16

///number of i2c devices in the table
#define I2C_DEVS_N 11
///number of sensors in the sensor table
#define SENSORS_N 24

//sensor numbers
#define THERMOCOUPLE_1 0
#define THERMOCOUPLE_2 1

/*
#define ADT7461E_1     8
#define ADT7461E_2     9

#define ADT7461D_1     16
#define ADT7461D_2     17
*/

#define SENSOR_OPEN (-1000*100)

///Struct to store i2c register table information
typedef struct {
	///temp high byte register
    int						reg_rtemph;
    ///temp low byte register
    int						reg_rtempl;
    ///number of limits
    int						reg_nlimith;
    ///array of limits
    int*					reg_wlimith;
    ///reg of limits
    int						reg_rlimith;
    ///device status register
    int						reg_rstatus;
    ///device id register
    int						reg_rid;
} t_regTable;

struct sensor_t;

///i2c device information
typedef struct {
	///how long a conversion takes
    int						convTime;
    ///temp offset
    int						offset;
    ///device address
    int 					addr;
    ///device id
    int 					deviceId;
    ///min temp value
    int						minThres;
    ///max temp value
    int 					maxThres;
    ///associated text
    const char* 	text;
    ///pointer to the reg table
    t_regTable*				pRegTable;
    /**
    	function pointer to function for data processing
    	\param sense sensor
    	\param low low byte
    	\param high high byte
    	\return temp
    */
    int (*process_data)(struct sensor_t *sense, int low, int high);
} i2c_info;

///holder for port information and device information
typedef struct {
	///port number
	I2CPORTID port;
	///pointer to the device information
	i2c_info *i2c;	
} i2c_full_info;

///spi device infromation
typedef struct {
	///chipselect
    unsigned int spi_cs;
    ///rx buffer
    unsigned short rx[2];
    ///tx buffer
    unsigned short tx[2];
    ///transaction callback
    void (*call)(spi_transfer *trans);
} spi_info;

/*
 Sensors have the following properties
 	-Name (for displaying in serial logs)
 	-Shortname (for displaying on LCD)
 	-Value (last measured value, stored as int value * 100)
 	-Time (last time, tick, that value was measured at)
 	-priv (pointer to private sensor data, like i2c info, or spi info, etc)
 And the following functions
 	-update (updates value, may just queue an update, returns 0 on success, non-zero on failure)
 		not required if sensor auto updates
 */
///struct to hold all sensor information
typedef struct sensor_t {
	///name of the sensor
    char *name;
    ///short name of sensor (for LCD)
    char *shortname;
    ///current value (stored as temp * 100)
    signed int value;
    ///last time sensor was updated (update must set this)
    portTickType time;
    ///union of all the private informations
    union {
    	///generic private information, here just for future use
        const void *priv_generic;
        ///i2c device information
        const i2c_full_info *priv_i2c;
        ///spi device information
        spi_info *priv_spi;
    };
	///update function, must update the sensor and return 0 on success
    unsigned int (*update)(struct sensor_t *);
    ///sensor priority, used in the sensor selection process
    unsigned char priority;
    ///checks if the sensor is connected
    unsigned char (*connected)(struct sensor_t *);
} sensor;

enum {DRIVER_MODE_HEAT, DRIVER_MODE_COOL, DRIVER_MODE_OFF};

/*
 Drivers have the following properties
 	-name
 	-mode (one of the above enums), reflects current driver state
 Functions
 	-heaton (if NULL indicates the device cannot heat)
 	-coldon (if NULL indicates the device cannot cool)
 	-turnoff (cannot be NULL)
*/
///holds all the information about the drivers
typedef struct driver_t {
	///name for printing
    const char *name;
    ///current mode
   unsigned int mode;
    ///channel (IE 1 or 2)
   unsigned int channel; //used as private data for driver
    ///call back to turn heat on (MUST BE DEFINED for a heater)
    void (*heaton)(struct driver_t *, unsigned short percent);
    ///call back to turn cold on (MUST BE DEFINED for a fan)
    void (*coldon)(struct driver_t *, unsigned short percent);
    ///call back to turn device off (MUST BE DEFINED)
    void (*turnoff)(struct driver_t *);
} driver;

/*
 Groups have the following properties
 	-Name
 	-sensor
 	-driver(s)
 	-target temp (int value * 100)
 	-threshold (int value * 100)
 	-mode
 	-safety time (set to 0 to disable safety timer, else set to number of milliseconds)
 */
///groups link the sensor and drivers
typedef struct group_t {
	///name of the group
    const char *name;
    ///sensor in use
    sensor *sens;
    ///target tempature
    signed int target_temp;
    ///threshold for error
    signed int threshold_temp;
    ///current group mode
    unsigned int mode;
    /// current max temp
    signed int max_temp;         //add
    /// current min temp
    signed int min_temp;          //add
    /// current coefficient
    unsigned int ramp_coefficient;       //add
    //over time
    unsigned int over_time;       //add
    //temp step
    signed int temp_step;     // chenglei add
    ///current safety timer
    unsigned int safety_timer;
    ///current safety degree rise
    signed int safety_degree;

	///the heater in use
    driver *heater;
    ///the fan in use
    driver *fan;
	///last error that occured
    unsigned int last_error;
    ///pid p term
    signed int pid_kp;
    ///pid i term
    signed int pid_ki;
    ///pid d term
    signed int pid_kd;
    ///pwm
    signed int pid_pwm;
	///pid mode
    signed int pid_mode;
     ///PID last error value
	signed int errL;
	
	unsigned int fan_speed;
	signed int threshold_fan;
    signed int last_pid_pwm;
	unsigned short userdefpid;
	float      slope;
    float      offset;	
} group;

/*
	Errors
 */
#define SENSOR_NO_ERROR 0
#define SENSOR_READ_ERROR 1
#define SENSOR_TIMEOUT_ERROR 2
#define SENSOR_SLOPE_ERROR 3
#define SENSOR_THRESHOLD_ERROR 4
#define SENSOR_NC_ERROR 5

extern const char *sensor_error_names[];

/*
	Sensor tables
 */
//extern sensor sensors[];
extern const i2c_info i2c_devices[];
extern driver drivers[];
/*
	Now define the primary groups that will be used
 */
extern group group1;
extern group group2;

/*
	functions to operate on a group, use these
	to add a layer of abstraction so we don't
	screw up access, most implemented inline to
	reduce calling overhead (but keep abstraction)
 */
///define used to change inline method if recompiling with different optimizations
#define INLINE_FUNCTION extern inline

INLINE_FUNCTION const char *get_sensor_name(group *g);
INLINE_FUNCTION const char *get_sensor_short_name(group *g);
INLINE_FUNCTION signed int get_sensor_temp(group *g);
INLINE_FUNCTION float  get_group_slope(group *g);
INLINE_FUNCTION void  set_group_slope(group *g,float s);
INLINE_FUNCTION float  get_group_offset(group *g);
INLINE_FUNCTION void  set_group_offset(group *g,float s);
INLINE_FUNCTION signed int get_group_target_temp(group *g);
INLINE_FUNCTION unsigned int get_group_mode(group *g);
INLINE_FUNCTION unsigned int get_heat_driver_mode(group *g);
INLINE_FUNCTION unsigned int get_fan_driver_mode(group *g);
extern unsigned int set_fan_driver(group *g,unsigned int i);
INLINE_FUNCTION void set_group_target(group *g, signed int target_temp);
INLINE_FUNCTION signed int get_group_threshold(group *g);
INLINE_FUNCTION signed int set_group_threshold(group *g, signed int thresh);
INLINE_FUNCTION void set_group_mode(group *g, unsigned int mode);
INLINE_FUNCTION void set_group_safety(group *g, unsigned int timer);
INLINE_FUNCTION unsigned int get_group_safety(group *g);
INLINE_FUNCTION signed int get_group_safety_degree(group *g);
INLINE_FUNCTION void set_group_safety_degree(group *g, signed int degree);
INLINE_FUNCTION signed int get_update_sensor_temp(group *g);
extern void sensor_error(group *g, unsigned int errno);
INLINE_FUNCTION unsigned int get_sensor_error(group *g);
INLINE_FUNCTION const char *get_sensor_error_text(group *g);
INLINE_FUNCTION void clear_sensor_error(group *g);

INLINE_FUNCTION signed int get_group_max_temp(group *g);
INLINE_FUNCTION void set_group_max_temp(group *g, signed int max_temp);
INLINE_FUNCTION signed int get_group_min_temp(group *g);
INLINE_FUNCTION void set_group_min_temp(group *g, signed int min_temp);
INLINE_FUNCTION signed int get_group_coefficient(group *g);
INLINE_FUNCTION void set_group_coefficient(group *g, unsigned int coefficient);
INLINE_FUNCTION signed int get_group_over_time(group *g);
INLINE_FUNCTION void set_group_over_time(group *g, unsigned int over_time);
INLINE_FUNCTION signed int get_group_temp_step(group *g); //chenglei add
INLINE_FUNCTION void set_group_temp_step(group *g, signed int temp_step); //chenglei add

INLINE_FUNCTION unsigned int get_group_fan_speed(group *g);
INLINE_FUNCTION void set_group_fan_speed(group *g , unsigned int fan_speed);
INLINE_FUNCTION unsigned int get_group_threshold_fan(group *g);
INLINE_FUNCTION unsigned int set_group_threshold_fan(group *g, signed int thresh);

INLINE_FUNCTION unsigned int get_group_fan_speed(group *g) {
 return g->fan_speed;
}
INLINE_FUNCTION void set_group_fan_speed(group *g , unsigned int fan_speed) { 
	g->fan_speed= fan_speed;
}
///set the group threshold
INLINE_FUNCTION unsigned int set_group_threshold_fan(group *g, signed int thresh) {
	g->threshold_fan = thresh;
}
INLINE_FUNCTION unsigned int get_group_threshold_fan(group *g) {
    return g->threshold_fan;
}

///get the sensor name
INLINE_FUNCTION const char *get_sensor_name(group *g) {
    return g->sens->name;
}

///get the short sensor name
INLINE_FUNCTION const char *get_sensor_short_name(group *g) {
    return g->sens->shortname;
}

///update the sensor temp and get the temp
INLINE_FUNCTION signed int get_update_sensor_temp(group *g) {
    //trigger update
    if ((g->sens->update)&&(g->sens->update(g->sens)))
    	sensor_error(g, SENSOR_READ_ERROR);
    return (signed int)(g->sens->value/100.0 *g->slope + g->offset)*100;
}

///get last measured sensor temp
INLINE_FUNCTION signed int get_sensor_temp(group *g) {
	return (signed int)(g->sens->value/100.0 *g->slope + g->offset)*100;
}

INLINE_FUNCTION void set_group_slope(group *g,float s) {
	g->slope = s;
}
INLINE_FUNCTION float get_group_slope(group *g) {
	return g->slope ;
}

INLINE_FUNCTION void set_group_offset(group *g,float s) {
	g->offset = s ;
}
INLINE_FUNCTION float get_group_offset(group *g) {
	return g->offset ;
}
///get group target temp
INLINE_FUNCTION signed int get_group_target_temp(group *g) {
    return g->target_temp;
}

///get group mode
INLINE_FUNCTION unsigned int get_group_mode(group *g) {
    return g->mode;
}

///get the mode of the heater
INLINE_FUNCTION unsigned int get_heat_driver_mode(group *g) {
    return g->heater->mode;
}

///get the mode of the fan
INLINE_FUNCTION unsigned int get_fan_driver_mode(group *g) {
    return g->fan->mode;
}

///set the group target temp
INLINE_FUNCTION void set_group_target(group *g, signed int target_temp) {
    g->target_temp = target_temp;
}

///get the group theshold
INLINE_FUNCTION signed int get_group_threshold(group *g) {
    return g->threshold_temp;
}

///set the group mode
INLINE_FUNCTION void set_group_mode(group *g, unsigned int mode) {
	if (mode == MODE_PELT || mode == MODE_RAMP || mode == MODE_RAMP_QC ){
		g->heater = &drivers[PELT_1 + (g == &group1 ? 0 : 1)];
		g->fan = &drivers[ FAN_1 + (g == &group1 ? 0 : 1)];
	}
	else{
		g->heater = &drivers[HEAT_1 + (g == &group1 ? 0 : 1)];
		g->fan = &drivers[ FAN_1 + (g == &group1 ? 0 : 1)];
	}

	    /*if (get_group_mode(S_GROUP1) == MODE_FAN_PWM)
	    {
	    	//set_sensor(S_GROUP1,ADT7461E_1);
	    	set_fan_driver(S_GROUP1,FANPWM1);
	    }
	    if (get_group_mode(S_GROUP2) == MODE_FAN_PWM)
	    {
	    	//set_sensor(S_GROUP2,ADT7461E_2);
	    	set_fan_driver(S_GROUP2,FANPWM2);
	    }*/
	if ( mode == MODE_PWM_QC){  //Tim change code style 

		g->heater = &drivers[HEAT_1 + (g == &group1 ? 0 : 1)];
		g->fan = &drivers[ FANPWM1+ (g == &group1 ? 0 : 1)];
	}
    g->mode = mode;
}

///set the group safety timer
INLINE_FUNCTION void set_group_safety(group *g, unsigned int timer) {
    g->safety_timer = timer;
}

///get the group safety timer
INLINE_FUNCTION unsigned int get_group_safety(group *g) {
    return g->safety_timer;
}

///get group safety rise amount
INLINE_FUNCTION signed int get_group_safety_degree(group *g) {
    return g->safety_degree;
}

///set group safety rise amoutn
INLINE_FUNCTION void set_group_safety_degree(group *g, signed int degree) {
    g->safety_degree = degree;
}

///set the group threshold
INLINE_FUNCTION signed int set_group_threshold(group *g, signed int thresh) {
	g->threshold_temp = thresh;
}

///get the last error
INLINE_FUNCTION unsigned int get_sensor_error(group *g) {
	return g->last_error;
}

///get the text associated with the last error
INLINE_FUNCTION const char *get_sensor_error_text(group *g) {
	return sensor_error_names[get_sensor_error(g)];
}

///clear the error from a channel
INLINE_FUNCTION void clear_sensor_error(group *g) {
	g->last_error = SENSOR_NO_ERROR;
	if ((group1.last_error == SENSOR_NO_ERROR)&&(group2.last_error == SENSOR_NO_ERROR)) {
		LED_ERROR_OFF;
	}
}

///get the pwm
INLINE_FUNCTION signed int get_group_pwm(group *g) {
    return ((g->pid_pwm)/100);
}

///get the errL
INLINE_FUNCTION signed int get_group_errL(group *g) {
    return g->errL;
}
///set the errL
INLINE_FUNCTION void set_group_errL(group *g, signed int errL){
	  g->errL = errL;
} 
///sets the mode
INLINE_FUNCTION void set_group_pid_mode(group *g, signed int mode) {
    g->pid_mode = mode;
}

///sets the kp
INLINE_FUNCTION void set_group_pid_kp(group *g, signed int kp) {
    g->pid_kp = kp;
}

///sets the ki
INLINE_FUNCTION void set_group_pid_ki(group *g, signed int ki) {
    g->pid_ki = ki;
}
///sets the kd
INLINE_FUNCTION void set_group_pid_kd(group *g, signed int kd) {
    g->pid_kd = kd;
}
///gets the pid_mode
INLINE_FUNCTION signed int get_group_pid_mode(group *g) {
    return g->pid_mode;
}

///gets the kp
INLINE_FUNCTION signed int get_group_pid_kp(group *g) {
    return g->pid_kp;
}

///gets the ki
INLINE_FUNCTION signed int get_group_pid_ki(group *g) {
    return g->pid_ki;
}
///gets the kd
INLINE_FUNCTION signed int get_group_pid_kd(group *g) {
    return g->pid_kd;
}
///get max temp
INLINE_FUNCTION signed int get_group_max_temp(group *g){
	  return g->max_temp;
}
///set max temp
INLINE_FUNCTION void set_group_max_temp(group *g, signed int max_temp){
	  g->max_temp = max_temp;
}
///get min temp
INLINE_FUNCTION signed int get_group_min_temp(group *g){
    return g->min_temp;
}
///set min temp
INLINE_FUNCTION void set_group_min_temp(group *g, signed int min_temp){
    g->min_temp = min_temp;
}
///get temp coefficient
INLINE_FUNCTION signed int get_group_coefficient(group *g){
	  return g->ramp_coefficient; 
}   
///set group coefficient
INLINE_FUNCTION void set_group_coefficient(group *g, unsigned int coefficient){
    g->ramp_coefficient = coefficient;
}

///get temp over time 
INLINE_FUNCTION signed int get_group_over_time(group *g){
	  return g->over_time; 
}   
///set group over time
INLINE_FUNCTION void set_group_over_time(group *g, unsigned int over_time){
    g->over_time = over_time;
}
///get temp step
INLINE_FUNCTION signed int get_group_temp_step(group *g){
    return g->temp_step;
}
///set temp step
INLINE_FUNCTION void set_group_temp_step(group *g, signed int temp_step){
    g->temp_step = temp_step;
}

extern void set_sensor(group *g, int i);
void perform_group_scan_update(group *g);
extern short sensor_status[];

#endif

