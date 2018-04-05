/// \file Control.c Main control code thread
/** \file
	Main control code, reads sensors, turns on/off the heater/cooler and fans.
	This is the only thread that calls get_update_sensor_temp
*/

#include "control.h"
#include "sensors.h"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

///give +/-0.5 degree band around target
#define STABLE_BAND 50
#define LED_STABLE_BAND 50

// Integral factor in PI controller
#define KI 5 //orig = 5
// Proportional factor in PI controller
#define KP 50 //orig = 50

///how long the temp must be in 2xSTABLE_BAND for the "stable" light to turn on (in ms)
#define STABLE_TIME 10000
//pin direction for FAN1
#define FAN1_SPD (1 << 28)
//pin direction for FAN2
#define FAN2_SPD (1 << 25)
unsigned int fan_counter1=0;
unsigned int fan_counter2=0;
unsigned int fan_counter3=0;
unsigned int fan_counter4=0; 
unsigned int fan_g1=0;
unsigned int fan_g3=0;
unsigned int fan_g2=0;
unsigned int fan_g4=0;

unsigned int flowMeter1Optional = 1;
unsigned int flowMeter2Optional = 1;
unsigned int sec_flag1=0 , sec_flag2=0;
///Holds the last time the temp left the "stable band"
portTickType lastTimeOutOfBand[2] = {0, 0};

///Holds the task ID of the control task (for suspending and resuming)
xTaskHandle  controlTask = NULL;
xTaskHandle  pullTask = NULL;
static int incrRatio(signed int last , signed int new ,double *ret) ;
static int pwm_adjust(signed int last, signed int *new ) ;
void wake_control_thread(void) {
	if (controlTask != NULL)
		vTaskResume(controlTask);

	if (pullTask != NULL)
		vTaskResume(pullTask);

}

signed int integral[2];
///Runs control code on a given sensor group
/**
Runs the control code on the sensor group, has on off control for fans and heaters, and PI control for Peltiers.
PI controller for Pelt mode has only gone through simple tests, may not be 100%.
PI controller also may have to be tuned for the power level of the Pelt you are using.
Currently the PI controller is tuned for a 100W Pelt @ 12v.

This function also updates the sensor in the group specified.
**NOTE** pwm is as percentage in this file. Which has a capped range from -100 to 100.
\param g which group to operate on
*/

signed int flag 	= 0;
unsigned int count 	= 0;
unsigned int count2 = 0;
//unsigned int fan_flag1=0;
//portTickType Time;

static int times; 
void control_sensor_group(group *g) {
	signed int temp = get_update_sensor_temp(g);
	signed int target = get_group_target_temp(g);
	signed int diff = temp - target; // different temp 
	signed int mode=get_group_mode(g);
	unsigned int num= get_group_coefficient(g) / 200;
	unsigned int timer= get_group_over_time(g) / 200;
	signed int temp_step = get_group_temp_step(g);
	
	temp_step = abs(temp_step);

	// check
	if(mode == MODE_RAMP || mode == MODE_RAMP_QC)
	{ 
		// we use below formula to adjust target temp to make it move the temperature point when increace and decreace;
		if (temp_step != 0)
		{
			target = get_group_min_temp(g) + (((target - get_group_min_temp(g)) / temp_step) * temp_step);
		}
		else
		{
			target = target;	// always use same target temp
		}
		
		// bounds check
		if (target >= get_group_max_temp(g))
		{
			target = get_group_max_temp(g);
		}
		else if (target <= get_group_min_temp(g))
		{
			target = get_group_min_temp(g);
		}

		// we caculate new target temp and then set it.
		set_group_target(g,target);

		count2 = count2 + 1;
		if(diff < STABLE_BAND && diff > -STABLE_BAND)
			count = count + 1;
	}

	if(count == num||count2 >= timer) // check stable count with coefficient number and count timer with over time.
	{
		if(((mode == MODE_RAMP) && (flag == 0)) || ((mode == MODE_RAMP_QC) && (flag == 0))) // lower than target temp
		{
			//target = target + 100; // increace target temp
			target = target + temp_step;	// chenglei add

			if(target >= get_group_max_temp(g))
			{
				flag = 1;
				target = get_group_max_temp(g);
			}
			set_group_target(g, target);

			// Time=Time+coefficient;
			count = 0;	// reset count
			count2 = 0;	// reset count2
		}
		else if((mode == MODE_RAMP && (flag == 1)) || (mode == MODE_RAMP_QC && (flag == 1)))	// higher than target temp
		{
			//target = target - 100; // decreace target temp
			target = target - temp_step;	// chenglei add
			
			if(target <= get_group_min_temp(g))
			{  
				flag = 0;
				target = get_group_min_temp(g);
			}
			set_group_target(g, target); 
			count = 0;
			count2 = 0;
		}
		else
		{

		}
	}

	//	signed int diff = temp - target;
	signed int P;
	signed int I;
	signed int D;
	signed int set_pwm;
	signed int tsit = 1;
	//  signed int intg = 0;;
	//	int i;
	signed int errL = get_group_errL(g);
	//integrate error
	//control signal not pegged (anti-windup)
	if (((g->pid_pwm > -99) || (diff > 0)) && ((g->pid_pwm < 99) || (diff < 0)))
		integral[(g == S_GROUP1) ? 0 : 1] += diff;

	//Cap the integral. This will help decrease overshoot
	if (integral[(g == S_GROUP1) ? 0 : 1] >= 1000)
		integral[(g == S_GROUP1) ? 0 : 1] = 1000;
	if (integral[(g == S_GROUP1) ? 0 : 1] <= -1000)
		integral[(g == S_GROUP1) ? 0 : 1] = -1000;

	// for version 1.3 and before: pid_mode: 0=manual, 1=225W, 2=350W, 3=225W Fast, 4= 225W Water Safety, 5= 350W Water Safety
	// for version 1.4 and later:  pid_mode: 0= 225W Water Safety ,1 = 350W Water Safety
	// due to pid_mode(225w safety and 350w safety) only exist on thermal header, so I add a condition here for FAN PWM control
	//
	if ( g->userdefpid == 0 ) {
		if (target <= 2500) // target temp less than 25 C
		{
    		//if (g->pid_mode==2||g->pid_mode==5)
			if (g->pid_mode == 1)
			{
    	    	//350W Head
				g->pid_kp = -72*target/100 + 2000;
				g->pid_ki = -8*target/100 + 300;
				g->pid_kd = g->pid_kd;
			}
    		//else if (g->pid_mode==1 || g->pid_mode==3||g->pid_mode==4)
			else if (g->pid_mode == 0)
			{
    	    	//225W Head
				g->pid_kp = -68*target/100 + 2000;
				g->pid_ki = -8*target/100 + 300;
				g->pid_kd = g->pid_kd;
			}
			else
			{
			}
		}
		else // target temp bigger than 25 C
		{
   			// if (g->pid_mode==2||g->pid_mode==5)
			if (g->pid_mode == 1)
			{
    	    	//350W Head
				g->pid_kp = 23*target/100-383;
				g->pid_ki = 5*target/100-42;
				g->pid_kd = g->pid_kd;
			}
   			// else if (g->pid_mode==1 || g->pid_mode==3 ||g->pid_mode==4)
			else if (g->pid_mode==0)
			{
    	    	//225W Head
				g->pid_kp = 30*target/100-417;
				g->pid_ki = 5*target/100-8;
				g->pid_kd = g->pid_kd;
			}
			else
			{
			}
		}
	}
    //g->pid_ki /= 2;
    //g->pid_kp += 50;
    //g->pid_ki += 50;
// Tim to log ATUO mode data heating or cooling procedure
	
	//Tim add for MODE_PWM_QC
	if ( g->userdefpid == 0 ) { // userdefpid == 1 
		if (get_group_mode(g) == MODE_PWM_QC)
		{
			// g->pid_kp = get_group_pid_kp(g);
			// g->pid_ki = get_group_pid_ki(g);
			// g->pid_kd = get_group_pid_kd(g);
			// debug 


   			g->pid_kp = (107*target/100 + 3602) * 2;
   			g->pid_ki = 10.6*target/100 + 160;
			g->pid_kd = g->pid_kd;
		}
	}
	P = (diff * (g->pid_kp))/100/100;
	I = (integral[(g == S_GROUP1) ? 0 : 1])*(g->pid_ki)/100/100;
	D = (g->pid_kd * (diff - errL)) /100/100;
	//set_pwm = (P + I);
	set_pwm = (P + I + D);

	set_group_errL(g, diff);

	if (set_pwm > 100)
		set_pwm = 100;
	if (set_pwm < -100)
		set_pwm = -100;

	g->last_pid_pwm = g->pid_pwm;
	if (get_group_mode(g) == MODE_PELT || get_group_mode(g) == MODE_RAMP)
	{
    	//if ((g->pid_pwm>0 && set_pwm<0) || (g->pid_pwm<0 && set_pwm>0))
    	//if (abs(g->pid_pwm-set_pwm)>50 && abs(diff)>5)
		if (abs(diff)>300) // diff temp bigger than 3 C
		{
       		// if (temp>=7000 && set_pwm>0 && g->pid_mode==3)
			if (temp >= 7000 && set_pwm > 0) // current temp >= 70 C and set_pwm > 0
			{
				if (g->pid_pwm/100 > set_pwm)
					g->pid_pwm -= 50;
				if (g->pid_pwm/100 < set_pwm)
					g->pid_pwm += 50;
			}
			else
			{
				if (g->pid_pwm/100 > set_pwm)
					g->pid_pwm -= 50;
				if (g->pid_pwm/100 < set_pwm)
					g->pid_pwm += 50;
			}
		}
		else if (temp > 5000)	// current temp  > 50 C
		{
			if (g->pid_pwm/100 > set_pwm)
				g->pid_pwm -= (abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100;
			if (g->pid_pwm/100 < set_pwm)
				g->pid_pwm += ((abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100 < abs(g->pid_pwm) / 10 + 50) ? 
							   (abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100 : abs(g->pid_pwm) / 10 + 50;
		}
		else if (temp < 1000)	// current temp < 10 C
		{
			if (g->pid_pwm/100 > set_pwm)
				g->pid_pwm -= ((abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100 < abs(g->pid_pwm) / 10 + 50) ? 
							   (abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100 : abs(g->pid_pwm) / 10 + 50;
			if (g->pid_pwm/100 < set_pwm)
				g->pid_pwm += (abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100;
		}
		else	//  10 <= current temp <= 50
		{
			if (g->pid_pwm/100 > set_pwm)
				g->pid_pwm -= ((abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100 < abs(g->pid_pwm) / 10 + 50) ? 
							   (abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100 : abs(g->pid_pwm) / 10 + 50;
			if (g->pid_pwm/100 < set_pwm)
				g->pid_pwm += ((abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100 < abs(g->pid_pwm) / 10 + 50) ? 
			 				   (abs(set_pwm - g->pid_pwm/100) + 2) / 3 * 100 : abs(g->pid_pwm) / 10 + 50;
		}
	}
    /*else if ((get_group_mode(g)==MODE_IDLE) && (get_group_last_mode(g)==MODE_PELT))
    {
        if (g->pid_pwm/100 > 0)
                g->pid_pwm-=50;
        if (g->pid_pwm/100 < 0)
            g->pid_pwm+=50;
    }*/
	else if (get_group_mode(g) == MODE_PWM_QC ) // PWM setting in user mode 
	{
		g->pid_pwm = set_pwm *100;
	}
	else if(get_group_mode(g) == MODE_PWM_QC && g->userdefpid == 0)
	{
		if ( g->pid_pwm * set_pwm < 0) {
			tsit = -1;
		}
		if(diff < STABLE_BAND && diff > -STABLE_BAND) {
			// keep it still 
			if ( tsit > 0 ) {
				g->pid_pwm = set_pwm * 100;
				pwm_adjust(g->last_pid_pwm,&g->pid_pwm);
			}
			else{ 
				if (abs(g->pid_pwm) <= 1000) 
				  g->pid_pwm = 0;
			  else
			    g->pid_pwm = g->pid_pwm*8/10;
			}

			if (g->pid_pwm > 9000) { g->pid_pwm = 9000;}
			if (g->pid_pwm < -9000 ) { g->pid_pwm = -9000;}
		}else if ( ((2 * STABLE_BAND > diff) && (diff > STABLE_BAND)) ||
				   	((-1 * STABLE_BAND > diff )&& (diff > - 2* STABLE_BAND)) ) {
			if ( tsit < 0) {
				 if( set_pwm < 0 ) {  // this case g->pid_pwm > 0
					g->pid_pwm = g->pid_pwm/5;  // it is cooling ,need heating 
					/*if (g->pid_pwm < 1800)
					   g->pid_pwm = 1800;
					   */
					if ( abs(g->pid_pwm) < 7) 
					     g->pid_pwm = 0;
				}else {
					g->pid_pwm = g->pid_pwm/2;
				}
			}else {
				g->pid_pwm = set_pwm * 100;
				pwm_adjust(g->last_pid_pwm,&g->pid_pwm);
			}
			if (g->pid_pwm > 9000) { g->pid_pwm = 9000;}
			if (g->pid_pwm < -9000 ) { g->pid_pwm = -9000;}
		}else { 
			g->pid_pwm = set_pwm * 100;
		}
	}
    else
    {
    	g->pid_pwm = 0;
    }

	//Debug PI controller
	//Keeps track of when we fall out of the stable band
    if (abs(diff) > (2 * LED_STABLE_BAND)) {
    	lastTimeOutOfBand[(g == S_GROUP1) ? 0 : 1] = xTaskGetTickCount();
    }

    switch(get_group_mode(g)){
    	case MODE_AUTO:
			if (diff > STABLE_BAND) { //we are hotter than target
				g->heater->turnoff(g->heater); // turn heater off
				g->fan->coldon(g->fan, 100); // turn fan on
			}
			else if (diff < -STABLE_BAND) { //we are cooler than target
				g->heater->heaton(g->heater, 100); // turn heater on
				g->fan->turnoff(g->fan); // turn fan off
			}
			else {//we are equal to set point
				g->heater->turnoff(g->heater); // heater off
				g->fan->turnoff(g->fan); // fan off
			}
			break;
		case MODE_PELT:
			if (g->pid_pwm > 0) { //we are hotter than target
				g->heater->coldon(g->heater, abs(g->pid_pwm/100*9/10)); // turn heater cold on
				g->fan->coldon(g->fan, 100); // turn fan on
			}
			else if (g->pid_pwm < 0) { //we are cooler than target
				//Peltiers heat up faster than they cool so to equalize the control loop we limit the heat power to half desired duty
				g->heater->heaton(g->heater, abs(g->pid_pwm/100*9/10));
				g->fan->coldon(g->fan, 100);
			}
			else{
				//not needed if we are doing PWM
				//g->heater->turnoff(g->heater);
				g->fan->coldon(g->fan, 100);
			}
			break;
		case MODE_RAMP:
			if (g->pid_pwm > 0) { //we are hotter than target
				g->heater->coldon(g->heater, abs(g->pid_pwm/100*9/10));
				g->fan->coldon(g->fan, 100);
			}
			else if (g->pid_pwm < 0) { //we are cooler than target
				//Peltiers heat up faster than they cool so to equalize the control loop we limit the heat power to half desired duty
				g->heater->heaton(g->heater, abs(g->pid_pwm/100*9/10));
				g->fan->coldon(g->fan, 100);
			}
			else{
				//not needed if we are doing PWM
				//g->heater->turnoff(g->heater);
				g->fan->coldon(g->fan, 100);
			}
			break;
		case MODE_FANON: //Heaters should always be off and fan should always be on
			if (diff > STABLE_BAND) { //we are hotter than target
				g->heater->turnoff(g->heater);
				g->fan->coldon(g->fan, 100);
			}
			else if (diff < -STABLE_BAND) { //we are cooler than target
				g->heater->turnoff(g->heater);
				g->fan->coldon(g->fan, 100);
			}
			else {//we are equal to set point
				g->heater->turnoff(g->heater);
				g->fan->coldon(g->fan, 100);
			}
			break;
		case MODE_RAMP_QC:
			if (diff > STABLE_BAND) { //we are hotter than target
				g->heater->turnoff(g->heater);
				g->fan->coldon(g->fan, 100);
			}
			else if (diff < -STABLE_BAND) { //we are cooler than target
				g->heater->heaton(g->heater, 100);
				g->fan->turnoff(g->fan);
			}
			else {//we are equal to set point
				g->heater->turnoff(g->heater);
				g->fan->turnoff(g->fan);
			}
			break;
		case MODE_PWM_QC:
//debug
			if (g->pid_pwm > 0) { //we are hotter than target
				//g->heater->coldon(g->heater, abs(g->pid_pwm/100*9/10)); // turn heater cold on
				//g->heater->turnoff(g->heater); // turn heater off	
				g->fan->coldon(g->fan, abs(g->pid_pwm/100));
				g->heater->turnoff(g->heater);
			}
			else if (g->pid_pwm < 0) { //we are cooler than target
				//Peltiers heat up faster than they cool so to equalize the control loop we limit the heat power to half desired duty
				g->fan->turnoff(g->fan);
				g->heater->heaton(g->heater, abs(g->pid_pwm/100));
			}
			else{
				g->fan->turnoff(g->fan);
				g->heater->turnoff(g->heater);
			}
			break;
		default: //Shut everything off, we should never reach this case
		{
			g->heater->turnoff(g->heater);
			g->fan->turnoff(g->fan);
		}
	}
}

unsigned char is_stable(group *g) {
	return (xTaskGetTickCount() - lastTimeOutOfBand[(g == S_GROUP1) ? 0 : 1]) > STABLE_TIME;
}


int cycle=0, cycle_one=0, cycle_two=0;
void vPollingTask(void *pvParameters) 
{
	vTaskDelay( portTICK_RATE_MS * 500 );	
	vTaskSetApplicationTaskTag(NULL, (void *)6);
	pullTask = xTaskGetCurrentTaskHandle();
	unsigned int pre_g1=0, cur_g1=0 ,pre_g2=0, cur_g2=0;
	unsigned int count_flag=0;
	while(1)
	{
		/* Tim: move this block to sensor.h  set_group_mode 
	    if (get_group_mode(S_GROUP1) == MODE_PWM_QC)
	    {
	    	//set_sensor(S_GROUP1,ADT7461E_1);
	    	set_fan_driver(S_GROUP1,FANPWM1);
	    }
	    if (get_group_mode(S_GROUP2) == MODE_PWM_QC)
	    {
	    	//set_sensor(S_GROUP2,ADT7461E_2);
	    	set_fan_driver(S_GROUP2,FANPWM2);
	    }*/

		//	if(get_group_pid_mode(S_GROUP1)==4 || get_group_pid_mode(S_GROUP1) ==5|| get_group_pid_mode(S_GROUP2)==4 || get_group_pid_mode(S_GROUP2) ==5)
		if(	get_group_pid_mode(S_GROUP1) == 0 ||
			get_group_pid_mode(S_GROUP1) == 1 || 
			get_group_pid_mode(S_GROUP2) == 0 || 
			get_group_pid_mode(S_GROUP2) == 1 ) 
		{
			if((AT91C_BASE_PIOA->PIO_PDSR & FAN1_SPD) == 0)
			{
				fan_counter1++;
				if(fan_counter1 >= 0xFFFF)
					fan_counter1 = 0;
			}

			pre_g1 = cur_g1;
			cur_g1 = AT91C_BASE_PIOA->PIO_PDSR & FAN1_SPD;

			if(!pre_g1 && cur_g1) // count the high and low level switch counter of group1.
			{
				fan_g1++;
				if(fan_g1 >= 0xFFFF)
					fan_g1 = 0;

    			/*   cycle_two= count_flag;
       			cycle=cycle_two-cycle_one;
       			cycle_one=cycle_two;
       
       			if(count_flag>=0xFFFF)
       			{
       				count_flag=0;
       			} */
       		}

   			if((AT91C_BASE_PIOA->PIO_PDSR & FAN2_SPD) == 0)
   			{
   				fan_counter3++;
   				if(fan_counter3 >= 0xFFFF)
   					fan_counter3 = 0;
   			}  

   			pre_g2=cur_g2;
   			cur_g2= AT91C_BASE_PIOA->PIO_PDSR & FAN2_SPD; 
   			if(!pre_g2&&cur_g2)	// count the high and low level switch counter of group2.
   			{
   				fan_g3++;
   				if(fan_g3 >= 0xFFFF)
   					fan_g3 = 0;
   			} 
   			vTaskDelay(2 * portTICK_RATE_MS);
			//count_flag++;
   		} 
   	}
}

void vControlTask(void *pvParameters) {
    int i;
	//delay a little, then start up the control (wait for full inititialization)
    vTaskDelay( portTICK_RATE_MS * 500 );

	//scan for sensors, select the appropriate one (that is the one with highest priority that is connected)
	//ensure that only BUS1 sensors are autoselected for BUS1 and BUS2 sensors for BUS2 (this can be overridden)
   	perform_group_scan_update(S_GROUP1);
   	perform_group_scan_update(S_GROUP2);

   	vTaskSetApplicationTaskTag(NULL, (void *)7);
   	controlTask = xTaskGetCurrentTaskHandle();
 	//------------add-------------- 

	//   AT91C_BASE_PIOA->PIO_ODR = 0x12000000;
   	AT91C_BASE_PIOA->PIO_PPUER = 0x12000000;                 // pull pa28 & pa25 to high by default


 	//--------------------------------------------------
	//Init integral array
   	integral[0] = 0;
   	integral[1] = 0;

   	int margin=15;
   	unsigned int r1 = 1, r2 = 1;
   	int blinking_flag = -1, blinking_flag2 = -1;
   	unsigned int fan1[60] = {0} , fan2[60] = {0} ;
   	unsigned int fan_num1 = 0 ,fan_num2 = 0;

   	while (1) {
   		fan_counter2 = fan_counter1;
   		fan_counter4 = fan_counter3;
   		
   		if(sec_flag1 == 0)
   		{	
   			fan_g2 = fan_g1;
			//	r1=1;
   		}
   		
   		if(sec_flag2 == 0)
   		{
   			fan_g4 = fan_g3;
			//	r2=1;
   		}
		//check group 1

		//debug
   		control_sensor_group(S_GROUP1);

		//check group 2
   		control_sensor_group(S_GROUP2);

		//update leds
   		if ((get_heat_driver_mode(S_GROUP1) != DRIVER_MODE_OFF)||(get_heat_driver_mode(S_GROUP2) != DRIVER_MODE_OFF))
   			LED_HEATON_ON;
   		else
   			LED_HEATON_OFF;

   		if ((get_fan_driver_mode(S_GROUP1) != DRIVER_MODE_OFF)||(get_fan_driver_mode(S_GROUP2) != DRIVER_MODE_OFF))
   			LED_FANON_ON;
   		else
   			LED_FANON_OFF;

   		if (((get_group_mode(S_GROUP1) == MODE_IDLE)||(is_stable(S_GROUP1)))
   			&&((get_group_mode(S_GROUP2) == MODE_IDLE)||(is_stable(S_GROUP2)))
   			&&((get_group_mode(S_GROUP1) != MODE_IDLE)||(get_group_mode(S_GROUP2) != MODE_IDLE)	)
   			)
   			LED_STABLE_ON;
   		else
   			LED_STABLE_OFF;

		vTaskDelay(200 * portTICK_RATE_MS); //XXX: Must be this large in order for thermocouples to read properly (they have 0.22s max conversion time)
											//		This should be fixed so that only the thermocouples update this slow

  		//  set_group_fan_speed(S_GROUP1, 60*1000/(cycle*2));


    	//  count fan speed group 1-------------------------------------------------
		sec_flag1++;
    	//int round1 = 5*r1;
		if( sec_flag1 == 5) // 5 * 200ms = 1 s
		{ 	
			sec_flag1 = 0;
			fan_num1  = 0;
			if(fan_g1 > fan_g2)
			{
				fan1[r1 % 60] = fan_g1 - fan_g2;
				
				int i = 0;

				for(i = 0; i < 60; i++) 
				{
					fan_num1 = fan_num1 + fan1[i];
				}

				if(fan1[0] == 0)
					set_group_fan_speed(S_GROUP1, (fan_num1/r1 + 0.5) * 60);
				else
					set_group_fan_speed(S_GROUP1, fan_num1);

    	 		//set_group_threshold_fan(S_GROUP1, r1%60);
			}
			else if(fan_g1 < fan_g2)
			{
				fan1[r1 % 60] = (fan_g1 + (0xFFFF - fan_g2));
				
				int j = 0;
				
				for(j = 0; j < 60; j++) 
				{
					fan_num1 += fan1[j];
				}

				if(fan1[0] == 0)
					set_group_fan_speed(S_GROUP1, (fan_num1/r1 + 0.5) * 60);
				else
					set_group_fan_speed(S_GROUP1, fan_num1);

			}
			else
			{
				set_group_fan_speed(S_GROUP1, 0);
			}	

			if(r1 == 60)
			{
				r1 = 0;
			}
			r1++;

			if( (get_group_mode(S_GROUP1) == MODE_PELT || get_group_mode(S_GROUP1) == MODE_RAMP) &&  // add check for RAMP mode by chenglei
				(get_group_pid_mode(S_GROUP1) == 0 || get_group_pid_mode(S_GROUP1) == 1) )   
			{ 
				unsigned int fan_speed1_min = get_group_fan_speed(S_GROUP1) ;
				if(fan_speed1_min < (get_group_threshold_fan(S_GROUP1)-margin))
				{
					//set_group_mode(S_GROUP1, MODE_IDLE);
					/*set_group_mode(S_GROUP1, MODE_FANON);// change default mode to MODE_FANON  Jackie*/
					if(flowMeterOptional_get(1))
					  LED_ERROR_ON;
					fan_g1 = 0;
					fan_g2 = 0;
    				//set_group_fan_speed(S_GROUP1, 0);
				}
				else if(fan_speed1_min >= (get_group_threshold_fan(S_GROUP1)-margin) && (fan_speed1_min < get_group_threshold_fan(S_GROUP1)))
				{
					blinking_flag = blinking_flag *(-1);
    	 			//	set_group_fan_speed(S_GROUP2, blinking_flag);	
					if(blinking_flag)				
					{	
						LED_ERROR_OFF; 
    	 				//LED_STABLE_ON;	
					}							
					if(blinking_flag == -1)
					{ 
						/*LED_ERROR_ON;	*/
						if(flowMeterOptional_get(1))
						  LED_ERROR_ON;
					}
				}
				else
				{
					LED_ERROR_OFF;
				}
			}
		}	
		
		if((get_group_mode(S_GROUP1) == MODE_PELT || get_group_mode(S_GROUP1) == MODE_RAMP) &&	// add check for RAMP mode by chenglei
		   (get_group_pid_mode(S_GROUP1) == 0 || get_group_pid_mode(S_GROUP1) == 1))
		{ 		
			if(fan_counter2 == fan_counter1)
			{
				//set_group_mode(S_GROUP1, MODE_IDLE);
				/*set_group_mode(S_GROUP1, MODE_FANON);// change default mode to MODE_FANON*/
				/*LED_ERROR_ON;*/
				if(flowMeterOptional_get(1))
				  LED_ERROR_ON;
				fan_counter2 = 0;
				fan_counter1 = 0; 
				set_group_fan_speed(S_GROUP1, 0);
			}   
		} 

    	// count fan speed group 2------------------------------------------------
		sec_flag2++;
   		//	int round2 = 5*r2;
		if( sec_flag2 == 5)
    	//if(sec_flag==5)
		{ 	
     		// if(r2==20)
			sec_flag2 = 0;
			fan_num2 = 0;
			if(fan_g3 > fan_g4)
			{
				fan2[r2%60] = fan_g3 - fan_g4;
				int i = 0;

				for(i = 0; i < 60; i++) 
				{
					fan_num2 = fan_num2 + fan2[i];
				}
				if(fan2[0] == 0)
					set_group_fan_speed(S_GROUP2, (fan_num2/r2+0.5)*60);
				else
					set_group_fan_speed(S_GROUP2, fan_num2);

    	 	 	//set_group_threshold_fan(S_GROUP2, r2%60);   
			}
			else if(fan_g3 < fan_g4)
			{
    			//set_group_fan_speed(S_GROUP2, (fan_g3+(0xFFFF -fan_g4))/r2 *60);
				fan2[r2%60] = (fan_g3 + (0xFFFF - fan_g4));
				int j = 0;
				for(j = 0; j<60; j++) 
				{
					fan_num2 += fan2[j];
				}
				if(fan2[0] == 0)
					set_group_fan_speed(S_GROUP2, (fan_num2/r2+0.5)*60);
				else
					set_group_fan_speed(S_GROUP2, fan_num2);
			}
			else
				set_group_fan_speed(S_GROUP2, 0);

			if(r2 == 60)
			{
				r2 = 0;
			}

			r2++;

			if( (get_group_mode(S_GROUP2) == MODE_PELT || get_group_mode(S_GROUP2) == MODE_RAMP) &&	// add check for RAMP mode by chenglei
			    (get_group_pid_mode(S_GROUP2) == 0 || get_group_pid_mode(S_GROUP2) == 1) )   
			{ 
				unsigned int fan_speed2_min= get_group_fan_speed(S_GROUP2) ;
				if(fan_speed2_min < get_group_threshold_fan(S_GROUP2) - margin)
				{
					//set_group_mode(S_GROUP2, MODE_IDLE);
					/*set_group_mode(S_GROUP2, MODE_FANON);// change default mode to MODE_FANON  for jackie*/
					/*LED_ERROR_ON; jackie*/
					if(flowMeterOptional_get(2))
					  LED_ERROR_ON;
					fan_g3=0;
					fan_g4=0;
					set_group_fan_speed(S_GROUP2, 0);
				}
				else if(fan_speed2_min>= (get_group_threshold_fan(S_GROUP2) - margin) && (fan_speed2_min < get_group_threshold_fan(S_GROUP2)))
				{
					blinking_flag2 = blinking_flag2 *(-1);
					if(blinking_flag2)				
					{	
						LED_ERROR_OFF; 
					}							
					if(blinking_flag2==-1)
					{ 
						if(flowMeterOptional_get(2))
						  LED_ERROR_ON;
						/*LED_ERROR_ON;	*/
					}
				}
				else
				{
					LED_ERROR_OFF;
				}
			}
		}	

		if( (get_group_mode(S_GROUP2) == MODE_PELT || get_group_mode(S_GROUP2) == MODE_RAMP) && // add check for RAMP mode by chenglei
		    (get_group_pid_mode(S_GROUP2) == 0 || get_group_pid_mode(S_GROUP2) == 1))   
		{ 		
			if(fan_counter4==fan_counter3)
			{
				//set_group_mode(S_GROUP2, MODE_IDLE);
				set_group_mode(S_GROUP2, MODE_FANON);// change default mode to MODE_FANON
				/*LED_ERROR_ON;*/  //jackie
				if(flowMeterOptional_get(2))
				  LED_ERROR_ON;
				fan_counter3=0;
				fan_counter4=0; 
				set_group_fan_speed(S_GROUP2, 0);
			}   	    	      
		} 
	}  
}
static int pwm_adjust(signed int last, signed int *new ) {
	double ret;
	if ( incrRatio(last , *new ,&ret) == 1 ) {
		if ( last != 0 && abs(ret)> 0.3) {
			if ( ret > 0 )
			  *new = last + last*0.3 ;
			else 
			  *new = last - last*0.3 ;
		}
		return 1; // adjust the new  
	}
	return 0; // keep it 


}

static int incrRatio(signed int last , signed int new ,double *ret) {
	if ( last * new > 0 ) {
		*ret = (new - last)*1.0/last; 
		return 1;
	} else if ( last  == 0 ) {
		return 0;
	} 

	return 0;

}

void flowMeterOptional_clear(unsigned int channel){
   if (channel == 1)
	 flowMeter1Optional = 0;
   else if(channel == 2)
	 flowMeter2Optional = 0;
}
void flowMeterOptional_set(unsigned int channel){
   if (channel == 1)
	 flowMeter1Optional = 1;
   else if(channel == 2)
	 flowMeter2Optional = 1;
}
unsigned int flowMeterOptional_get(unsigned int channel){
	if (channel == 1)
	  return flowMeter1Optional;
	else if(channel == 2)
	  return flowMeter2Optional;
}
#include "AT91SAM7X256.h"
/*#include "../Source/portable/GCC/ARM7_AT91SAM7S/lib_AT91SAM7X256.h"*/

__inline void _RSTSoftReset(
        AT91PS_RSTC pRSTC,
        unsigned int reset)
{
	pRSTC->RSTC_RCR = (0xA5000000 | reset);
}
void reset(void){

	_RSTSoftReset(AT91C_BASE_RSTC,0x0000000D);

}

