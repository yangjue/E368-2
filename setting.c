
#include "setting.h"
#include "Drivers\eeprom.h"


GroupSetting setting[2];

void resetgroup(group * gp, GroupSetting *gs)
{
	gp->target_temp = gs->target_temp;
	gp->threshold_temp = gs->threshold_temp;
	gp->mode = gs->mode;
	gp->max_temp = gs->max_temp;
	gp->min_temp = gs->min_temp;
	gp->ramp_coefficient = gs->ramp_coefficient;
	gp->over_time = gs->over_time;
	gp->temp_step = gs->temp_step;
	gp->safety_degree = gs->safety_degree;
	gp->safety_timer = gs->safety_timer;
	gp->pid_kp = gs->pid_kp;
	gp->pid_ki = gs->pid_ki;
	gp->pid_kd = gs->pid_kd;
	gp->pid_mode = gs->pid_mode;
	gp->threshold_fan = gs->threshold_fan;
	gp->userdefpid = gs->userdefpid;
	gp->slope = gs->slope;
	gp->offset = gs->offset;

	return;
}
int	savesetting(group *gp)
{
	unsigned int s = 1;
	int i;
	char *c;
	if (gp == S_GROUP1)
	  s = 0;
	setting[s].target_temp = gp->target_temp;
	setting[s].threshold_temp = gp->threshold_temp;
	setting[s].mode = gp->mode;
	setting[s].max_temp = gp->max_temp;
    setting[s].min_temp = gp->min_temp;	
	setting[s].ramp_coefficient = gp->ramp_coefficient;
	setting[s].over_time = gp->over_time;
	setting[s].temp_step = gp->temp_step;
	setting[s].safety_timer = gp->safety_timer;
	setting[s].safety_degree = gp->safety_degree;
	setting[s].pid_kp = gp->pid_kp;
	setting[s].pid_ki = gp->pid_ki;
	setting[s].pid_kd = gp->pid_kd;
	setting[s].pid_mode = gp->pid_mode;
	setting[s].threshold_fan = gp->threshold_fan;
	setting[s].userdefpid = gp->userdefpid;
	setting[s].slope = gp->slope;
	setting[s].offset = gp->offset;
	store(s,(char *)&setting[s],(unsigned char) sizeof(setting));  
	c = (char *)&setting[s];

	return 1;
}
int reloadsetting(unsigned int chan)
{
	fetch(chan,(unsigned char) sizeof(GroupSetting),(char*)&setting[chan]);
	if (chan == 0) 
	  resetgroup(S_GROUP1, &setting[0]);
	else 
	  resetgroup(S_GROUP2, &setting[1]);
	return 1;
}
