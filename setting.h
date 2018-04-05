#ifndef _SETTING_H
#define _SETTING_H
#include "sensors.h"

typedef struct {
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
    unsigned int ramp_coefficient; //add
    //over time
    unsigned int over_time;       //add
    //temp step
    signed int temp_step;     // chenglei add
    ///current safety timer
    unsigned int safety_timer;
    ///current safety degree rise
    signed int safety_degree;
    ///pid p term
    signed int pid_kp;
    ///pid i term
    signed int pid_ki;
    ///pid d term
    signed int pid_kd;
    ///pwm
    //signed int pid_pwm;
	///pid mode
    signed int pid_mode;
	
	signed int threshold_fan;
	unsigned short userdefpid;
	float      slope;
    float      offset;	
}__attribute__((packed)) GroupSetting;

int	savesetting(group *gp);
int reloadsetting(unsigned int chan);
void resetgroup(group * gp, GroupSetting *gs);
#endif 
