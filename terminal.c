/// \file terminal.c terminal(serial) interface
/** \file
	terminal interface that runs on all serial interfaces.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#include "board.h"
#include "terminal.h"
#include "Drivers/SPI/spiDriver.h"
#include "lcdInterface.h"

#include "sensors.h"
#include "Drivers/I2C/i2cDriver.h"

#include "control.h"
#include "eeprom.h"
#include "setting.h"
#include "safety.h"
#include "lcdInterface.h"

#define TRUE 1
#define FALSE 0

///terminal buffer, used for various string manipulation
static char termBuffer[128];

short firsterror1 = 0;
short lasterror1 = 0;
char errorLog1[4][30]={0};
short firsterror2 = 0;
short lasterror2 = 0;
char errorLog2[4][30]={0};
#define COMMANDTABLE_N (sizeof(commandTable)/sizeof(t_command))
///table of commands that may be used in the terminal
static t_command commandTable[] = {
    { "watch12",    cmd_watch12, 		  "watch12 - Refresh the current sensor temp for group 1 and 2\r\n" },
    { "watch1",     cmd_watch1, 		  "watch, watch1 - Refresh the current sensor temp for group 1\r\n"},
    { "watch2",     cmd_watch2, 		  "watch2 - Refresh the current sensor temp for group 2\r\n" },
    { "watch",      cmd_watch1, 		  NULL},

    { "settemp1",   cmd_settemp1,     "settemp,settemp1 [temp] - Sets the target temperature for group 1\r\n" },
    { "settemp2",   cmd_settemp2,     "settemp2 [temp] - Sets the target temperature for group 2\r\n" },
    { "settemp", 	  cmd_settemp1,     NULL },

    { "gettemp1",   cmd_gettemp1,     "gettemp,gettemp1 - Gets the target temperature for group 1\r\n" },
    { "gettemp2",   cmd_gettemp2,     "gettemp2 - Gets the target temperature for group 2\r\n" },
    { "gettemp", 	  cmd_gettemp1,     NULL },

    { "setthresh1",	cmd_setthresh1,   "setthresh,setthresh1 [temp] - Sets the group 1 temperature threshold\r\n" },
    { "setthresh2",	cmd_setthresh2,   "setthresh2 [temp] - Sets the group 2 temperature threshold\r\n" },
    { "setthresh",	cmd_setthresh1,   NULL},

    { "setmode1",	  cmd_setmode1,     "setmode, setmode1 [auto|fanon|pelt|ramp|idle|rampqc|pwmqc] - Set group 1 feedback control system\r\n" },
    { "setmode2",	  cmd_setmode2,     "setmode2 [auto|fanon|pelt|ramp|idle|rampqc|pwmqc] - Set group 2 feedback control system\r\n" },
    { "setmode",	  cmd_setmode1,     NULL },

    { "safety1",	  cmd_safety1,	    "safety1 [on|off] - Turns on or off thermal runaway safety for group 1\r\n" },
    { "safety2",	  cmd_safety2,	    "safety2 [on|off] - Turns on or off thermal runaway safety for group 2\r\n" },

    { "setsafetyval1",	cmd_setsafetyval1,	"setsafetyval1 [sec,deg] - Second,Degree toward target temp showing no thermal runaway for group 1\r\n" },
    { "setsafetyval2",	cmd_setsafetyval2,	"setsafetyval2 [sec,deg] - Second,Degree toward target temp showing no thermal runaway for group 2\r\n" },

    { "tinfo1",   cmd_tinfo1,		"tinfo,tinfo1 - Displays current temperature information for group 1\r\n" },
    { "tinfo2",   cmd_tinfo2,		"tinfo2 - Displays current temperature information for group 2\r\n" },
    { "tinfo", 		cmd_tinfo1,		NULL },

    { "status1",    cmd_status1, 	"status,status1 - Display the current operation status for group 1\r\n" },
    { "status2",    cmd_status2, 	"status2 - Display the current operation status for group 2\r\n" },
    { "status",     cmd_status1, 	NULL },

    { "isstable1",	cmd_isstable1,	"isstable,isstable1 - Checks for temperature stability for group 1\r\n" },
    { "isstable2",	cmd_isstable2,	"isstable2 - Checks for temperature stability for group 2\r\n" },
    { "isstable",   cmd_isstable1,  NULL },

    //{ "g",		cmd_group1info, 	"group1info - Displays current information for group 1\r\n" },  //debug
    { "group1info",		cmd_group1info, 	"group1info - Displays current information for group 1\r\n" },
    { "group2info",		cmd_group2info, 	"group2info - Displays current information for group 2\r\n" },

    { "forcesen1", 	cmd_forcesen1,		"forcesen1 [1|2|3|4|5|6|7|8] - Force Temperature sensor1\r\n" },
    { "forcesen2", 	cmd_forcesen2,		"forcesen2 [1|2|3|4|5|6|7|8] - Force Temperature sensor2\r\n" },
    { "scan", 	cmd_scan,		        "scan - Scan Temperature sensor channel 1 and 2...\r\n" },

    { "help", 		cmd_help,		"help - Displays this message\r\n" },
    { "chkerr12", 	cmd_chkerr12,		"chkerr12 - Display and clear the error flag at group 1 & 2\r\n" },
    { "chkerr1", 	cmd_chkerr1,		"chkerr1 - Display and clear the error flag at group 1\r\n" },
    { "chkerr2", 	cmd_chkerr2,		"chkerr2 - Display and clear the error flag at group 2\r\n" },
    { "chkerr", 	cmd_chkerr1,		NULL },
    { "clrerr1", 	cmd_clrerr1,		"clrerr1 - Clears the error status of channel 1\r\n" },
    { "clrerr2", 	cmd_clrerr2,		"clrerr2 - Clears the error status of channel 2\r\n" },
    { "clrerr", 	cmd_clrerr1,		NULL },
    { "debug",		cmd_debug,			NULL },
	{ "version",	cmd_getver,			"version - Get firmware version number\r\n"},
	{ "setid",		cmd_setid,			"setid [ID] - saves the argument (8 characters) into the ID non-volatile memory\r\n"},
	{ "getid",		cmd_getid,			"getid - gets the ID from non-volatile memory\r\n"},
	{ "setmac",     cmd_setmac,         "setmac - XX:XX:XX:XX:XX:XX - sets the MAC address (must reboot to take effect)\r\n"},
	{ "getmac",     cmd_getmac,         "getmac - gets the current MAC address \r\n"},
	{ "flowmeter1opt",cmd_flowmeter1opt,"flowmeter1opt [1|0]- flow meter 1 optional 1 to set, 0 to clear \r\n"},
	{ "flowmeter2opt",cmd_flowmeter2opt,"flowmeter2opt [1|0]- flow meter 2 optional 1 to set, 0 to clear \r\n"},
	{ "pid1setopt",cmd_pid1setopt,"pid1setopt [1|0]-  En/Dis chan1 PID setting for user,arg 1 to enable, 0 to disable \r\n"},
	{ "pid2setopt",cmd_pid2setopt,"pid2setopt [1|0]-  En/Dis chan2 PID setting for user,arg 1 to enable, 0 to disable \r\n"},
	{ "E368Reset",cmd_reset,            "E368Reset: - Reset the E368\r\n"},
	//{ "dispip",		cmd_dispip,			"dispip - displays the current IP address\r\n"},
	//{ "getip",		cmd_getip,			"getip - gets the current IP in EEPROM (0.0.0.0 means DHCP assigned)\r\n"},
	//{ "setip",		cmd_setip,			"setip - sets the IP in EEPROM (0.0.0.0 means use DHCP)\r\n"}, 
	// { "setfan1pwm", cmd_setfan1pwm,		"setfan1pwm [pwm] - Set fan1 pwm\r\n"},
	// { "monitorfan", cmd_monitorfan,		"monitorfan - Monitor fan speed\r\n"},
	{ "monitorpid1", cmd_monitorpid1,		"monitorpid1 - Monitor1 PWM P x100 I x100 D x100\r\n"},
	{ "monitorpid2", cmd_monitorpid2,		"monitorpid2 - Monitor2 PWM P x100 I x100 D x100\r\n"},
	{ "MCUArch",cmd_endian,"MCUArch: - Pislay MCU endian\r\n"},
	{ "logSetting1",cmd_logchan1,"logSetting1 - log chan 1 setting\r\n"},
	{ "logSetting2",cmd_logchan2,"logSetting2 - log chan 2 setting\r\n"},
	{ "loadSetting1",cmd_loadsetting1,"loadSetting1 - load chan 1 setting\r\n"},
	{ "loadSetting2",cmd_loadsetting2,"loadSetting2 - load chan 2 setting\r\n"},
    //{ "setheatpid", cmd_setheatpid, 	"setheatpid [p,i,d]\r\n"},
	//{ "setfanpid",  cmd_setfanpid, 		"setfanpid [p,i,d]\r\n"},
	{ "setslope1",cmd_setslope1,"setslope1 - set chan 1 sensor slope\r\n"},
	{ "getslope1",cmd_getslope1,"getslope1 - get chan 1 sensor slope\r\n"},
	{ "setslope2",cmd_setslope2,"setslope2 - set chan 2 sensor slope\r\n"},
	{ "getslope2",cmd_getslope2,"getslope2 - get chan 2 sensor slope\r\n"},
	{ "setoffset1",cmd_setoffset1,"setoffset1 - set chan 1 sensor offset\r\n"},
	{ "setoffset2",cmd_setoffset2,"setoffset2 - set chan 2 sensor offset\r\n"},
	//{ "seterrorlog",seterrorlog,"seterrorlog - debug: eg-  chan  & str\r\n"},
	{ "geterrorlog1",cmd_geterrlog1,"geterrorlog1 - get chan 1 error logs\r\n"},
	{ "geterrorlog2",cmd_geterrlog2,"geterrorlog2 - get chan 2 error logs\r\n"},
	{ "input",cmd_input,"input - debug to set the error input\r\n"},
	{ "version",cmd_version,"version - return version of E368 Firmware\r\n"},
	//{ "dstore",cmd_dstore,"dstore -debug store \r\n"},
	//{ "dfetch",cmd_dfetch,"dfetch -debug fetch \r\n"},
	{ "thermaltype",cmd_thermaltype,"thermaltype -return each chan's thermal type\r\n"},

};
union D {
	GroupSetting g;
	char c[70];
};
void cmd_dstore(char *arg)
{
	union D d;
	int i = 0;
	for( i =0 ; i < 70 ; i ++) 
	  d.c[i] = i;
	store(0,(char *)&d.g,70);  
	store(1,(char *)&d.g,70);  
	return ;
}

void cmd_dfetch(char *args)
{
	union D d;
	int i = 0;
	unsigned int a;
    sscanf(args, "%u", &a);
	fetch(a,(unsigned char) sizeof(GroupSetting),(char*)&d.c[0]);
	for ( i = 0 ; i+4 < 70 ; i=i+5)
	  printf("data [%d]  %d  %d  %d  %d %d \r\n",i,d.c[i],d.c[i+1],d.c[i+2],d.c[i+3],d.c[i+4]);

	return;
}
void cmd_input(void)
{
	seterrorlog(1,"a1");
	seterrorlog(1,"a2");
	seterrorlog(1,"a3");
	seterrorlog(1,"a4");
	seterrorlog(1,"a5");
	seterrorlog(1,"a6");
	displayerror();

}
void displayBanner(void) {
    printf("     '##::: ##'##::::'##'####'########:'####:::'###::::\r\n");
    printf("      ###:: ##:##:::: ##. ##::##.... ##. ##:::'## ##:::\r\n");
    printf("      ####: ##:##:::: ##: ##::##:::: ##: ##::'##:. ##::\r\n");
    printf("      ## ## ##:##:::: ##: ##::##:::: ##: ##:'##:::. ##:\r\n");
    printf("      ##. ####. ##:: ##:: ##::##:::: ##: ##::#########:\r\n");
    printf("      ##:. ###:. ## ##::: ##::##:::: ##: ##::##.... ##:\r\n");
    printf("      ##::. ##::. ###:::'####:########:'####:##:::: ##:\r\n");
    printf("     ..::::..::::...::::....:........::....:..:::::..::\r\n");

    printf("          _____ _                 _____ _____ _____\r\n");
    printf("        |_   _| |_ ___ ___ _____| __  |     |_   _|\r\n");
    printf("          | | |   | -_|  _|     | __ -|  |  | | |\r\n");
    printf("          |_| |_|_|___|_| |_|_|_|_____|_____| |_|\r\n\r\n");
    printf("                        Version %s\r\n",E368_FIRMWARE_VERSION);
    printf("                Oregon (Amberwood) Edition\r\n\r\n");
}


void vTerminalTask(void *pvParameters) {
    char command[128];
    vTaskSetApplicationTaskTag(NULL, (void *)5);
    displayBanner();
    printf("Please enter a command or type \"help\" to list available commands\r\n\r\n");
    while(1) {
        printf("\r\nThermBot>");
        readline(&command);
        printf("\r\n");
        if(checkIfAllowedCommand(command)) {

        }
    }
}

int checkIfAllowedCommand(char * c) {
    int i;
    int arg;
    for(arg = 0; arg < strlen(c); arg++) {
		if (c[arg] == ' ') {//found end of command, break it here
			c[arg] = 0;
			arg++;
			break;
		}
    }
    for(i=0; i < COMMANDTABLE_N; i++) {
        if(strcmp(c, commandTable[i].command) == 0) {
            commandTable[i].function(c + arg);
            return 1;
        }
    }
    printf("Not a valid command, type \"help\" for a list of commands\r\n");
    return 0;
}

//------------------------------------------------------------------------------
/// Iterate terminal
///
//------------------------------------------------------------------------------
int get_number_from_argument(char *arg, signed int *temp) {
	//XXX: not exactly the fastest method :/
	float val;
	if ((strlen(arg) == 0)||(sscanf(arg, "%f", &val) == 0)) {
		printf("Invalid input, please enter a number\r\n");
		return 1;
	}
	*temp = (val * 100);
	return 0;
}


// Help Command
void cmd_help( char *arg ) {
    int i;
    printf("\r\nCommand - Description of command functionality\r\n");
    printf("-----------------------------------------------\r\n");
    for(i=0; i<COMMANDTABLE_N; i++) {
        if(commandTable[i].help != NULL) {
            printf("%s", commandTable[i].help);
        }
    }
}

/*void cmd_dispip( char *arg ) {
	printf("Current IP address: ");
	printIPAddress();
	printf("\r\n");
}*/

//Watch Commands
void cmd_watch12(char *arg) {
	portTickType startTime;
	portTickType timer;
    printf("Starting watch of Channel 1 and Channel 2\r\n");
    printf("Time (S)\tChannel 1 (C)\tChannel 2 (C)\r\n");
    startTime = xTaskGetTickCount();
    timer = xTaskGetTickCount();
    while (!serialBytesWaiting()) {
    	strcpy(termBuffer, "%i\t\t");
    	formatTemp(get_sensor_temp(S_GROUP1) , termBuffer + strlen(termBuffer));
    	strcat(termBuffer, "\t\t");
    	formatTemp(get_sensor_temp(S_GROUP2) , termBuffer + strlen(termBuffer));
    	strcat(termBuffer, "\r\n");
		printf(termBuffer, (xTaskGetTickCount() - startTime) / (portTICK_RATE_MS * 1000));
		vTaskDelayUntil(&timer, (1000 * portTICK_RATE_MS));
    }
    serialFlushInputBuffers();
}
void cmd_watch1(char *arg) {
	portTickType startTime;
	portTickType timer;
    printf("Starting watch of Channel 1\r\n");
    printf("Time (S)\tChannel 1 (C)\r\n");
    startTime = xTaskGetTickCount();
    timer = xTaskGetTickCount();
    while (!serialBytesWaiting()) {
    	strcpy(termBuffer, "%i\t\t");
    	formatTemp(get_sensor_temp(S_GROUP1) , termBuffer + strlen(termBuffer));
    	strcat(termBuffer, "\r\n");
		printf(termBuffer, (xTaskGetTickCount() - startTime) / (portTICK_RATE_MS * 1000));
		vTaskDelayUntil(&timer, (1000 * portTICK_RATE_MS));
    }
    serialFlushInputBuffers();
}
void cmd_watch2(char *arg) {
	portTickType startTime;
	portTickType timer;
    printf("Starting watch of Channel 2\r\n");
    printf("Time (S)\tChannel 2 (C)\r\n");
    startTime = xTaskGetTickCount();
    timer = xTaskGetTickCount();
    while (!serialBytesWaiting()) {
    	strcpy(termBuffer, "%i\t\t");
    	formatTemp(get_sensor_temp(S_GROUP2) , termBuffer + strlen(termBuffer));
    	strcat(termBuffer, "\r\n");
		printf(termBuffer, (xTaskGetTickCount() - startTime) / (portTICK_RATE_MS * 1000));
		vTaskDelayUntil(&timer, (1000 * portTICK_RATE_MS));
    }
    serialFlushInputBuffers();
}

//Set temp commands
void cmd_settemp(group *g, char *arg ) {
	int temp;
	if (get_number_from_argument(arg, &temp))
		return;
	strcpy(termBuffer, "Setting ");
	strcat(termBuffer, g->name);
	strcat(termBuffer, " target temp to ");
	formatTemp(temp, termBuffer + strlen(termBuffer));
	strcat(termBuffer, "\r\n");
    printf(termBuffer);
    set_group_target(g, temp);
}

void cmd_settemp1( char *arg ) {
    cmd_settemp(S_GROUP1, arg);
}

void cmd_settemp2( char *arg ) {
    cmd_settemp(S_GROUP2, arg);
}

//Get temp commands

void cmd_gettemp(group *g) {
	strcpy(termBuffer, "\t");
	formatTemp(get_sensor_temp(g) , termBuffer + strlen(termBuffer));
	strcat(termBuffer, "\r\n");
	printf(termBuffer);
}

void cmd_gettemp1() {
	cmd_gettemp(S_GROUP1);
}

void cmd_gettemp2() {
	cmd_gettemp(S_GROUP2);
}


void cmd_setthresh(group *g, char *arg ) {
	int temp;
	if (get_number_from_argument(arg, &temp))
		return;
	strcpy(termBuffer, "Setting ");
	strcat(termBuffer, g->name);
	strcat(termBuffer, " threshold temp to ");
	formatTemp(temp, termBuffer + strlen(termBuffer));
	strcat(termBuffer, "\r\n");
    printf(termBuffer);
    set_group_threshold(g, temp);
}

void cmd_setthresh1( char *arg ) {
    cmd_setthresh(S_GROUP1, arg);
}
void cmd_setthresh2( char *arg ) {
    cmd_setthresh(S_GROUP2, arg);
}

void cmd_setmode(group *g, char *arg) {
	if (strcmpi(arg, "auto") == 0) {
		set_group_mode(g, MODE_AUTO);
		printf("Set mode to AUTO\r\n");
	}
	else if (strcmpi(arg, "fanon") == 0) {
		set_group_mode(g, MODE_FANON);
		printf("Set mode to FANON\r\n");
	}
	else if (strcmpi(arg, "pelt") == 0) {
		set_group_mode(g, MODE_PELT);
		printf("Set mode to PELT\r\n");
	}
	else if (strcmpi(arg, "ramp") == 0) {
		set_group_mode(g, MODE_RAMP);
		printf("Set mode to RAMP\r\n");
	}
	else if (strcmpi(arg, "idle") == 0) {
		set_group_mode(g, MODE_IDLE);
		printf("Set mode to IDLE\r\n");
	}
	else if (strcmpi(arg, "rampqc") == 0) {
		set_group_mode(g, MODE_RAMP_QC);
		printf("Set mode to RAMP Qual Cooler\r\n");
	}
	else if (strcmpi(arg, "pwmqc") == 0) {
		set_group_mode(g, MODE_PWM_QC);
		printf("Set mode to PWM Qual Cooler\r\n");
	}
	else {
		printf("\"%s\" is not a known mode, try: auto, fanon, pelt, idle, rampqc\r\n", arg);
	}
}

void cmd_setmode1( char *arg ) {
    cmd_setmode(S_GROUP1, arg);
}

void cmd_setmode2( char *arg ) {
    cmd_setmode(S_GROUP2, arg);
}

void cmd_isstable(group *g, char *arg) {
    if (is_stable(g))
    	printf("%s is stable\r\n", g->name);
    else
    	printf("%s is unstable\r\n", g->name);
}

void cmd_isstable1( char *arg ) {
	cmd_isstable(S_GROUP1, arg);
}
void cmd_isstable2( char *arg ) {
    cmd_isstable(S_GROUP2, arg);
}
void cmd_tinfo(group *g, char *arg) {
	strcpy(termBuffer, "Current information for ");
	strcat(termBuffer, g->name);
	strcat(termBuffer, "\r\n");
	printf(termBuffer);
	
	strcpy(termBuffer, "Current Temperature:\t");
    formatTemp(get_sensor_temp(g) , termBuffer + strlen(termBuffer));
    strcat(termBuffer, "\r\n");
    printf(termBuffer);

    strcpy(termBuffer, "Current Threshold:\t");
    formatTemp(get_group_threshold(g) , termBuffer + strlen(termBuffer));
    strcat(termBuffer, "\r\n");
    printf(termBuffer);

    strcpy(termBuffer, "Target Temperature:\t");
    formatTemp(get_group_target_temp(g) , termBuffer + strlen(termBuffer));
    strcat(termBuffer, "\r\n");
    printf(termBuffer);
}

void cmd_tinfo1( char *arg ) {
    cmd_tinfo(S_GROUP1, arg);
}

void cmd_tinfo2( char *arg ) {
    cmd_tinfo(S_GROUP2, arg);
}

void cmd_status(group *g, char *arg) {
	strcpy(termBuffer, "Status of ");
	strcat(termBuffer, g->name);
	strcat(termBuffer, "\r\n");
	printf(termBuffer);

	strcpy(termBuffer, "Mode:        ");
	switch (get_group_mode(g)) {
		case MODE_AUTO:
			strcat(termBuffer, "AUTO");
		break;
		case MODE_FANON:
			strcat(termBuffer, "FANON");
		break;
		case MODE_IDLE:
			strcat(termBuffer, "IDLE");
		break;
		case MODE_PELT:
			strcat(termBuffer, "PELT");
		break;
		case MODE_RAMP:
			strcat(termBuffer, "RAMP");
		break;
		case MODE_RAMP_QC:
			strcat(termBuffer, "RAMPQC");
		break;
		default:

		break;
	}
	strcat(termBuffer, "\r\n");
	printf(termBuffer);

	strcpy(termBuffer, "Heater mode: ");
	switch (get_heat_driver_mode(g)) {
		case DRIVER_MODE_HEAT:
			strcat(termBuffer, "HEAT");
		break;
		case DRIVER_MODE_COOL:
			strcat(termBuffer, "COOL");
		break;
		case DRIVER_MODE_OFF:
			strcat(termBuffer, "OFF");
		break;
		default:

		break;
	}
	strcat(termBuffer, "\r\n");
	printf(termBuffer);
	
	strcpy(termBuffer, "Fan mode:    ");
	switch (get_fan_driver_mode(g)) {
		case DRIVER_MODE_HEAT:
			strcat(termBuffer, "HEAT");
		break;
		case DRIVER_MODE_COOL:
			strcat(termBuffer, "COOL");
		break;
		case DRIVER_MODE_OFF:
			strcat(termBuffer, "OFF");
		break;
		default:

		break;
	}
	strcat(termBuffer, "\r\n");
	printf(termBuffer);
}

void cmd_status1( char *arg ) {
	cmd_status(S_GROUP1, arg);
}
void cmd_status2( char *arg ) {
    cmd_status(S_GROUP2, arg);
}

void cmd_chkerr(group *g) {
	printf("%s error: %s\r\n", g->name, get_sensor_error_text(g));
}

void cmd_chkerr1( char *arg ) {
    cmd_chkerr(S_GROUP1);
}
void cmd_chkerr2( char *arg ) {
    cmd_chkerr(S_GROUP2);
}
void cmd_chkerr12( char *arg ) {
    cmd_chkerr(S_GROUP1);
    cmd_chkerr(S_GROUP2);
}

void cmd_clrerr1(char *arg) {
	printf("Cleared group 1 error\r\n");
	clear_sensor_error(S_GROUP1);
}

void cmd_clrerr2(char *arg) {
	printf("Cleared group 2 error\r\n");
	clear_sensor_error(S_GROUP2);
}

void cmd_scan(char *arg) {
	int i, j;
	for (j = I2C_PORT1; j <= I2C_PORT2; j++) {
		printf("Scanning I2CBUS %i\r\n", j+1);
		I2CScanBus(j);
		for (i = 0; i < I2C_DEVS_N; i++) {
			strcpy(termBuffer, "\t");
			strcat(termBuffer, i2c_devices[i].text);
			//strcat(termBuffer, "Test");
			strcat(termBuffer, "\t");
			if (I2CSensorFound(j, i))
				strcat(termBuffer, "[  FOUND  ]\r\n");
			else
				strcat(termBuffer, "[NOT FOUND]\r\n");
			printf(termBuffer);
		}
	}
}

void cmd_forcesen(group *g, char *arg) {
	int i = atoi(arg);
	if ((i < 1)||(i > SENSORS_N)) {
		printf("Please enter a number between 1 and %i.\r\n", SENSORS_N);
		return;
	}
	i--; //reduce by 1 because our array starts at 0
	set_sensor(g, i);
	printf("Changed %s to sensor %s\r\n", g->name, g->sens->name);
}

void cmd_forcesen1(char *arg) {
	cmd_forcesen(S_GROUP1, arg);
}
void cmd_forcesen2(char *arg) {
	cmd_forcesen(S_GROUP2, arg);
}

void cmd_safety(group *g, char *arg) {
	if (!strcmpi(arg, "on")) {
		//turn on safety to default settings
		set_group_safety(g, SAFETY_TIMER_DEFAULT);
		set_group_safety_degree(g, 100);
		printf("Safety turned on\r\n");
	}
	else if (!strcmpi(arg, "off")) {
		//turn off safety
		set_group_safety(g, 0);
		printf("Safety turned off\r\n");
	}
	else {
		printf("Please enter \"on\" or \"off\" to turn the safety on or off\r\n");
	}
}

void cmd_safety1(char *arg) {
	cmd_safety(S_GROUP1, arg);
}
void cmd_safety2(char *arg) {
	cmd_safety(S_GROUP2, arg);
}

void cmd_setsafetyval(group *g, char *arg) {
	float sec, deg;
	int time, temp;
	if (sscanf(arg, "%f, %f", &sec, &deg) != 2) {
		printf("improper arguments, please enter time, degrees\r\n");
		return;
	}
	time = sec * 1000;
	temp = deg * 100;
	set_group_safety(g, time);
	set_group_safety_degree(g, temp);
	strcpy(termBuffer, "Set safety timer to %i sec for ");
	formatTemp(temp, termBuffer + strlen(termBuffer));
	strcat(termBuffer, "\r\n");
	printf(termBuffer, time/1000);
}

void cmd_setsafetyval1(char *arg) {
	cmd_setsafetyval(S_GROUP1, arg);
}

void cmd_setsafetyval2(char *arg) {
	cmd_setsafetyval(S_GROUP2, arg);
}

void cmd_groupinfo(group *g, char *arg) {
	printf("%s\r\n=========================\r\n", g->name);
	printf("Sensor in use: %s\r\n", g->sens->name);
	printf("Heat driver:   %s\r\n", g->heater->name);
	printf("Fan driver:    %s\r\n\r\n", g->fan->name);
	printf("Fan threshold:    %d\r\n\r\n", g->threshold_fan);
	printf("Fan speed:    %d\r\n\r\n", g->fan_speed);
	printf("target_temp= %d  ,0x%x\r\n",g->target_temp,g->target_temp);
	printf("threshold_temp= %d ,0x%x\r\n" , g->threshold_temp, g->threshold_temp);
	printf("mode = %d ,0x%x\r\n" ,g->mode,g->mode);
	printf("max_temp = %d ,0x%x\r\n" ,g->max_temp,g->max_temp);
    printf("min_temp = %d ,0x%x\r\n" ,g->min_temp,g->min_temp);	
	printf("ramp_coefficient = %d ,0x%x\r\n" ,g->ramp_coefficient,g->ramp_coefficient);
	printf("over_time = %d ,0x%x\r\n",g->over_time,g->over_time);
	printf("temp_step =%d ,0x%x\r\n" ,g->temp_step,g->temp_step);
	printf("safety_timer = %d ,0x%x\r\n",g->safety_timer,g->safety_timer);
	printf("safety_degree = %d ,0x%x\r\n",g->safety_degree,g->safety_degree);
	printf("pid_kp =%d ,0x%x\r\n" ,g->pid_kp,g->pid_kp);
	printf("pid_ki =%d ,0x%x\r\n" ,g->pid_ki,g->pid_ki);
	printf("pid_kd =%d ,0x%x\r\n" ,g->pid_kd,g->pid_kd);
	printf("pid_mode = %d ,0x%x\r\n",g->pid_mode,g->pid_mode);
	printf("threshold_fan =%u ,0x%x\r\n", g->threshold_fan,g->threshold_fan);
	printf("IFUserDefpid = %u,0x%x\r\n",g->userdefpid,g->userdefpid);
	printf("SensorVal Slope = %d,0x%x\r\n",(int)(g->slope*100),g->slope);
	printf("SensorVal Offset = %d,0x%x\r\n",(int)(g->offset*100),g->offset);
	cmd_status(g, arg);
	printf("\r\n");
	cmd_tinfo(g, arg);	
	cmd_gettemp(g);
}

void cmd_group1info(char *arg) {

	cmd_groupinfo(S_GROUP1, arg);
	return ;
}
void cmd_group2info(char *arg) {

	cmd_groupinfo(S_GROUP2, arg);
	return;
}
//General function that can be used for debugging
void cmd_debug(char *arg) {
	printf("\r\n RSTC_RMR: 0x%x" , AT91C_BASE_RSTC->RSTC_RMR);
	printf("\r\n RSTC_RCR: 0x%x" , AT91C_BASE_RSTC->RSTC_RCR);
}

void cmd_getver(char *arg) {
	printf("Firmware Version: %s\r\n" , E368_FIRMWARE_VERSION);
}
void cmd_reset(char *arg){
	reset();
}
void cmd_flowmeter1opt(char *arg){
	int a = atoi(arg);
	if (a ==0 || a ==1){
		if( a == 0) flowMeterOptional_clear(1);
		if( a == 1) flowMeterOptional_set(1);
	}
	else 
		return;
}
void cmd_flowmeter2opt(char *arg){
	int a = atoi(arg);
	if (a ==0 || a ==1){
		if( a == 0) flowMeterOptional_clear(2);
		if( a == 1) flowMeterOptional_set(2);
	}
	else 
		return;
}
void cmd_pid1setopt(char *arg){
	int a = atoi(arg);
	if (a ==0 || a ==1){
		if( a == 0) group1.userdefpid = 0;
		if( a == 1) group1.userdefpid = 1;
	}
	else 
	  printf("Invalid input \r\n");
		return;
}
void cmd_pid2setopt(char *arg){
	int a = atoi(arg);
	if (a ==0 || a ==1){
		if( a == 0) group2.userdefpid = 0; //disable user define
		if( a == 1) group2.userdefpid = 1;
	}
	else 
	  printf("Invalid input \r\n");
		return;
}
void cmd_monitorpid(group *g,char *arg)
{
	portTickType timer;
	timer = xTaskGetTickCount();
	printf("Starting watch PWM, P, I, D, Tar, Cur\r\n");
    while (!serialBytesWaiting()) {
    	printf("PWM = %i, KP = %i, KI = %i, KD = %i ,Tar = %d ,Cur = %d\r\n", get_group_pwm(g),get_group_pid_kp(g),get_group_pid_ki(g),get_group_pid_kd(g),
					g->target_temp,get_sensor_temp(g));
		vTaskDelayUntil(&timer, (1000 * portTICK_RATE_MS));
    }
    serialFlushInputBuffers();
}

void cmd_monitorpid1(char *arg)
{
	cmd_monitorpid(S_GROUP1,arg);
}

void cmd_monitorpid2(char *arg)
{
	cmd_monitorpid(S_GROUP2,arg);
}
void cmd_endian(char *arg)
{
	union {
		char c[2];
		unsigned int i;
	}u;
	u.i = 0xFF11;
	if (u.c[0] == 0xFF)
	  printf("MCU is BigEndian\r\n");
    else 
	  printf("MCU is LittleEndian\r\n");
    printf("short:%u, int:%u, long:%u \r\n",sizeof(short),sizeof(int),sizeof(long));
	printf("float:%u \r\n",sizeof(float));
	printf("sizeof GroupSetting: %u \r\n",sizeof(GroupSetting));
}
void cmd_logchan1(char *arg)
{
	unsigned int l = savesetting(S_GROUP1);
}
void cmd_logchan2(char *arg)
{
	unsigned int l = savesetting(S_GROUP2);
}
void cmd_loadsetting1(char *arg)
{
	reloadsetting(0);
}
void cmd_loadsetting2(char *arg)
{
	reloadsetting(1);
}
void cmd_setslope1(char *arg)
{
	float val;
	char s[8];
	if ((strlen(arg) == 0)||(sscanf(arg, "%f", &val) == 0)) {
		printf("Invalid input, please enter a number\r\n");
		return ;
	}
	formatfloatstr(val, s);
	printf("set slope 1 : %s \r\n",s);

	setslope(S_GROUP1,val);
}
void cmd_getslope1()
{
	char s[8];
	group *g = S_GROUP1;
	formatfloatstr(g->slope, s);
	printf("slope 1 : %s \r\n",s);
}
void cmd_setslope2(char *arg)
{
	float val;
	char s[8];
	if ((strlen(arg) == 0)||(sscanf(arg, "%f", &val) == 0)) {
		printf("Invalid input, please enter a number\r\n");
		return ;
	}
	formatfloatstr(val, s);
	printf("set slope 2 : %s \r\n",s);

	setslope(S_GROUP2,val);
}
void cmd_getslope2()
{
	char s[8];
	group *g = S_GROUP2;
	formatfloatstr(g->slope, s);
	printf("slope 2 : %s \r\n",s);
}
void setslope(group *g,float f)
{
	if( fabs(f) > 0.0001) 
		g->slope = f;
	else 
	    g->slope = 0.0;
}
void cmd_setoffset1(char *arg)
{
	float val;
	char s[8];
	if ((strlen(arg) == 0)||(sscanf(arg, "%f", &val) == 0)) {
		printf("Invalid input, please enter a number\r\n");
		return ;
	}
	formatfloatstr(val,s);
	printf("set offset 1 : %s \r\n",s);
	setoffset(S_GROUP1,val);
}
void cmd_setoffset2(char *arg)
{
	float val;
	char s[8];
	if ((strlen(arg) == 0)||(sscanf(arg, "%f", &val) == 0)) {
		printf("Invalid input, please enter a number\r\n");
		return ;
	}
	formatfloatstr(val,s);
	printf("set offset 2 : %s \r\n",s);
	setoffset(S_GROUP2,val);
}
void setoffset(group *g,float f)
{
	g->offset = f;
}
void cmd_geterrlog1(void )
{
	geterrorlog(1);
	return;
}
void cmd_geterrlog2(void )
{
	geterrorlog(2);
	return;
}
void geterrorlog(unsigned short i)
{
	short firsterror = 0;
	short lasterror = 0;
	char  (*errorlog)[30];
	short j = 0;
	short t = 0;
	if (i == 1) { firsterror = firsterror1; lasterror = lasterror1; errorlog = &errorLog1[0][0];}
	if (i == 2) { firsterror = firsterror2; lasterror = lasterror2; errorlog = &errorLog2[0][0];}
	//printf("i : %d , firsterror : %d , lasterror :%d \r\n",i,firsterror,lasterror);
	if (lasterror == firsterror ) { return ;}
	if (lasterror > firsterror) {
		for ( j = lasterror-1 ; t < 10 && j != firsterror ; j = (j--)%4,t++ ) {
			//printf("i : %d , firsterror : %d , lasterror :%d j :%d\r\n",i,firsterror,lasterror,j);
			printf("Error Log: %s \r\n",errorlog[j]); 
		}
		//printf("i : %d , firsterror : %d , lasterror :%d j :%d\r\n",i,firsterror,lasterror,j);
		printf("Error Log: %s \r\n",errorlog[firsterror]); 
		printf("Error Log: %s \r\n",errorlog[firsterror-1]); 
	}else {
		for ( j = lasterror+4-1; t < 10 && j !=firsterror ; j--, t++) {
			//printf("i : %d , firsterror : %d , lasterror :%d j :%d\r\n",i,firsterror,lasterror,j);
			printf("Error Log: %s \r\n",errorlog[j%4]);
		}
		printf("Error Log: %s \r\n",errorlog[firsterror]); 
		printf("Error Log: %s \r\n",errorlog[firsterror-1]); 
	}


}
//void seterrorlog(unsigned short i,char * s)
void seterrorlog(short i , char *r)
{
	short *pfirsterror;
	short *plasterror ;
	char  s[20];
	char  (*errorlog)[30];
	strcpy(s,r);
	//printf("input: %s\r\n",s);
	if (i == 1) { pfirsterror = &firsterror1; plasterror = &lasterror1; errorlog = errorLog1;}
	if (i == 2) { pfirsterror = &firsterror2; plasterror = &lasterror2; errorlog = errorLog2;}
	strcpy(errorlog[*plasterror],s);
	(*plasterror)++;
	(*plasterror) = (*plasterror)%4;
	if (*plasterror == *pfirsterror ){
		(*pfirsterror)++;
		(*pfirsterror) = (*pfirsterror)%4;
	}
	//printf("chan 1 : first %d, last %d\r\n",firsterror1,lasterror1);
	//printf("chan 2 : first %d, last %d\r\n",firsterror2,lasterror2);
}
void formatfloatstr(float val, char *s )
{
	int v, i; 
	char* buf = s;
	v = val * 100;
	if (v < 0){
        s[0] = '-';
		buf = &s[1];
		itoa(-1*v,buf); }
	else 
	    itoa(v,buf);

    i = strlen( buf );
    if (i > 2) {
        buf[i] = buf[i-1];
        buf[i-1] = buf[i-2];
        buf[i-2] = '.';
        buf[i+1] = 0;
    } else if (i == 2) {
        buf[4] = 0;
        buf[3] = buf[1];
        buf[2] = buf[0];
        buf[0] = '0';
        buf[1] = '.';
    } else if (i == 1) {
        buf[4] = 0;
        buf[3] = buf[0];
        buf[2] = '0';
        buf[1] = '.';
        buf[0] = '0';
    }
	return ;

}
void cmd_version ( void ) {
	printf("E368 Firmware version: %s [Test]\r\n","D1.9");
}
void cmd_thermaltype ( char *arg ) {
	char *str[2];
	int i = 0 ;
	for ( i = 0 ; i < 2 ; i++) {
		if ( sensor_status[i] == 0 ) {
			str[i] = "Unknown";
		}
		if ( sensor_status[i] == 1 ) {
			if ( device_type[i] == 1 ) {
			str[i] = "QualCooler";
		} else {
			str[i] = "Peliter";
			}
		}
	}
	printf("chan 1: %s chan2: %s\r\n",str[0],str[1]);
}
