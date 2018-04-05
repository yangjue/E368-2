#include "safety.h"
#include "sensors.h"

#include "FreeRTOS.h"
#include "task.h"
#include "control.h"
#include "terminal.h"

/// \file safety.c Safety task


///the min update period, will error if sensor isn't updated within this time (in ms)
#define MIN_UPDATE_PERIOD	5000

///if the sensor hasn't updated in this time, then kick the control thread back to startup.
#define KICK_PERIOD			1000

#define GROUP_1_OUTPUT_TYPE_PIN 29
#define GROUP_2_OUTPUT_TYPE_PIN 15

///Safety timer start time
static portTickType safetyValTime[2] = {0, 0};
///Safety timer start value (temp)
static signed int 	safetyValTemp[2] = {0, 0};

unsigned char   device_type[2] = {0};
short sensor_status[2] = {0} ;
///Checks if a group sensor has updated recently
/**
	Checks that the group sensor has updated within the required time
	if not we might have a problem.

	\param g group to check
	\return 0 == problem, 1 == ok
*/
static unsigned char check_updated_recently(group *g) {
	if ((g->sens->time + (MIN_UPDATE_PERIOD * portTICK_RATE_MS)) < xTaskGetTickCount()) { //it has been more than the allowed update interval
		//this means that something has locked up, we could have a problem
		return 0;
	}
	if ((g->sens->time + (KICK_PERIOD * portTICK_RATE_MS)) < xTaskGetTickCount()) {
		wake_control_thread();
	}
	return 1;
}

///checks if the mode is AUTO or PELT
static unsigned char is_active(group *g) {
	return ((get_group_mode(g) == MODE_AUTO)||(get_group_mode(g) == MODE_PELT) || (get_group_mode(g) == MODE_RAMP) || (get_group_mode(g) == MODE_RAMP_QC) || (get_group_mode(g) == MODE_PWM_QC ));
}

///checks that the slope is ok
/**
	Checks that the safety timer has not found a fault.

	\param g group to check
	\param num number that corrisponds to the group (group1 == 0, etc)
	\return 0 == failure, 1 == ok
*/
static unsigned char slope_ok(group *g, unsigned char num) {
	if (safetyValTime[num] == 0) { //take initial measurement 
		safetyValTime[num] = g->sens->time;
		safetyValTemp[num] = get_sensor_temp(g);
	}
	else if ((safetyValTime[num] + get_group_safety(g)) <= xTaskGetTickCount()) { //we have hit the end of the measurement period
		signed int diff = get_sensor_temp(g) - safetyValTemp[num];
		if (diff < get_group_safety_degree(g))
			return 0; //safety failure
	}
	return 1; //all ok
}

///check that the sensor is not over the threshold
static unsigned char over_thresh(group *g) {
	return (get_sensor_temp(g) > get_group_threshold(g));
}

///check that a heater is connected (OUTPUTx_TYPE == 1)
static unsigned char heater_connected(group *g) {
	if (g == S_GROUP1)
		return (AT91C_BASE_PIOA->PIO_PDSR & (1 << GROUP_1_OUTPUT_TYPE_PIN)) ? 1 : 0;
	else
		return (AT91C_BASE_PIOA->PIO_PDSR & (1 << GROUP_2_OUTPUT_TYPE_PIN)) ? 1 : 0;
}

///safety task
/**
	The safety task runs and watches all sensors to make sure of the following things:
		-Sensors are updating appropriately
		-Safety timer hasn't been violated
		-Error threshold hasn't been violated
	Under any of those conditions an error is triggered and that group will be set to FANON
	mode.
*/
void vSafetyTask(void *pvParameters) {
	portTickType timer;
	group *g = S_GROUP1;
	int i = 0;
	unsigned char bus;
	
	//delay a little, then start up the safety (wait for full inititialization)
	vTaskDelay( portTICK_RATE_MS * 500 );
	
	//scan for sensors, select the appropriate one
	
	vTaskSetApplicationTaskTag(NULL, (void *)8);

	timer = xTaskGetTickCount();	
	while (1) {
		for (i = 0; i < 2; i++) {
			if (i == 0)
				g = S_GROUP1;
			else
				g = S_GROUP2;
			//check that the sensor is not NC while system is active
			if (get_sensor_temp(g)< -20000 || get_sensor_temp(g)> 200*100) {
				sensor_status[i] = 0;
			}else {
				sensor_status[i] = 1;
			}
			if ((is_active(g))&&(get_sensor_temp(g)==SENSOR_OPEN)) {
				//throw an error because this shouldn't be done...
				sensor_error(g, SENSOR_NC_ERROR);
				if (g = S_GROUP1){
					seterrorlog(1,sensor_error_names[SENSOR_NC_ERROR]);
				}else {
					seterrorlog(2,sensor_error_names[SENSOR_NC_ERROR]);
				}
			}
			if ( heater_connected(g)) {
				device_type[i] = 1;
			} else {
				device_type[i] =  0;
			}
			//check that we are not in pelt mode, and have a heater
			//connected, force into AUTO
			if ((get_group_mode(g) == MODE_PELT)&&(heater_connected(g))) {
				set_group_mode(g, MODE_AUTO);
			}
            //check that we are not in heater mode, and have a peltier
			//connected, force into peltier. Prevent driving peltier 100%
            if ((get_group_mode(g) == MODE_AUTO)&&(!heater_connected(g))) {
				set_group_mode(g, MODE_PELT);
			}
			//check that both sensors have been updated recently
			//if it hasn't been updated, then an error will be produced
			//if not in the idle state, or fanon state
			if ((is_active(g))&&(!check_updated_recently(g))) {
				//group timeout error
				sensor_error(g, SENSOR_TIMEOUT_ERROR);
				if (g = S_GROUP1){
					seterrorlog(1,sensor_error_names[SENSOR_TIMEOUT_ERROR]);
				}else {
					seterrorlog(2,sensor_error_names[SENSOR_TIMEOUT_ERROR]);
				}
			}
			//if safety is on, and heat is on store values for a while and check slope
			//if slope is below safety then error
			if ((is_active(g))&&(get_heat_driver_mode(g) == DRIVER_MODE_HEAT)&&(get_group_safety(g) != 0)) {
				//check slope of group1
				if (!slope_ok(g, i)) {
					//slope error
					sensor_error(g, SENSOR_SLOPE_ERROR);
					if (g = S_GROUP1){
						seterrorlog(1,sensor_error_names[ SENSOR_SLOPE_ERROR]);
					}else {
						seterrorlog(2,sensor_error_names[ SENSOR_SLOPE_ERROR]);
					}
				}
			}
			else {
				//clear slope mesurement
				safetyValTime[i] = 0;
				safetyValTemp[i] = 0;
			}

			//check that the group has not exceeded the threshold
			if (over_thresh(g)) {
				sensor_error(g, SENSOR_THRESHOLD_ERROR);
				if (g = S_GROUP1){
					seterrorlog(1,sensor_error_names[SENSOR_THRESHOLD_ERROR]);
				}else {
					seterrorlog(2,sensor_error_names[SENSOR_THRESHOLD_ERROR]);
				}
			}
		}
		//sleep for a while, we want this code to execute once a second
		vTaskDelayUntil(&timer, (1000 * portTICK_RATE_MS));
	}
}


