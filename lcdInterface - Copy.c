//------------------------------------------------------------------------------
//  _NVRM_COPYRIGHT_BEGIN_
//
//   Copyright 1993-2008 by NVIDIA Corporation.  All rights reserved.  All
//   information contained herein is proprietary and confidential to NVIDIA
//   Corporation.  Any use, reproduction, or disclosure without the written
//   permission of NVIDIA Corporation is prohibited.
//
//   _NVRM_COPYRIGHT_END_
//------------------------------------------------------------------------------
/// \file lcdInterface.c LCD user interface
/** \file
	contains all the workings for the LCD user interface
*/

// Dependencies
#include <stdlib.h>
#include <string.h>
#include "control.h"

#include "Board.h"
#include "lcdInterface.h"
#include "sensors.h"
#include "Drivers/LCD/lcdDriver.h"
#include "itoa.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "control.h"
//#include "Drivers/TCP/tcpSerial.h"
//#include "Drivers/EMAC/SAM7_EMAC.h"
//#define FAN1_SPD (1 << 28)
//pin direction for FAN2
//#define FAN2_SPD (1 << 25)

//unsigned int count_fan=0;
// Local interfaces
///Checks for a rising edge on a button
/**
	Checks the input and determines if interface should see a button press.
	This function will only trigger on a rising edge, then repeat condition

	\param pin value of the pin in question
	\param last_state pointer to a variable to use to hold the last state of the pin
	\param lastPush pointer to variable to hold the time of the last push (may
			be NULL if button is not to repeat)
*/
int risingEdge( int pin, int* last_state, portTickType* lastPush );

///Updates the display
void displayIterate( void );
///Checks for input and performs actions required
void buttonIterate( void );

///Callback funtions that draws the Home screen
static void LCDDrawHome( void );
///Callback function that handles input from buttons for channel select screen
static void LCDDrawGroupSelect( void );
///Callback funtion that draws the set target 1 screen
static void LCDDrawSetTgt1( void );
///Callback funtion that draws the set threshold 1 screen
static void LCDDrawSetThres1( void );
///Callback funtion that draws the set mode 1 screen
static void LCDDrawSetMode1( void );
///Callback funtion that draws the safety on/off screen
static void LCDDrawSetSafeOn1( void );
///Callback funtion that draws the set safety timer 1 screen
static void LCDDrawSetSafeTime1( void );
///Callback funtion that draws the set safety degree 1 screen
static void LCDDrawSetSafeDeg1( void );
///Callback funtion that draws the set target 2 screen
static void LCDDrawSetTgt2( void );
///Callback funtion that draws the set theshold 2 screen
static void LCDDrawSetThres2( void );
///Callback funtion that draws the set mode 2 screen
static void LCDDrawSetMode2( void );
///Callback funtion that draws the set safety on/off 2 screen
static void LCDDrawSetSafeOn2( void );
///Callback funtion that draws the set safety timer 2 screen
static void LCDDrawSetSafeTime2( void );
///Callback funtion that draws the set safety degree 2 screen
static void LCDDrawSetSafeDeg2( void );
///Callback funtion that draws the set PidMode
static void LCDDrawSetPidMode1( void );
///Callback funtion that draws the set P
static void LCDDrawSetP1( void );
///Callback funtion that draws the set I
static void LCDDrawSetI1( void );
///Callback funtion that draws the set D
static void LCDDrawSetD1( void );
///Callback funtion that draws the set PidMode
static void LCDDrawSetPidMode2( void );
///Callback funtion that draws the set P
static void LCDDrawSetP2( void );
///Callback funtion that draws the set I
static void LCDDrawSetI2( void );
///Callback funtion that draws the set D
static void LCDDrawSetD2( void );
///Callback funtion that draws the set max temperature 1 screen
static void LCDDrawSetMaxTemp1( void );             //add
///Callback funtion that draws the set min temperature 1 screen
static void LCDDrawSetMinTemp1( void );             //add
///Callback funtion that draws the set Ramping Coefficient  1 screen
static void LCDDrawSetCoefficient1( void );             //add
///Callback funtion that draws the set Ramping over time  1 screen
static void LCDDrawSetOverTime1( void );             //add
///Callback funtion that draws the set temperature step  1 screen
static void LCDDrawSetTempStep1( void );    //chenglei add
///Callback funtion that draws the set max temperature 2 screen
static void LCDDrawSetMaxTemp2( void );             //add
///Callback funtion that draws the set min temperature 2 screen
static void LCDDrawSetMinTemp2( void );             //add
///Callback funtion that draws the set Ramping Coefficient  2 screen
static void LCDDrawSetCoefficient2( void );             //add
///Callback funtion that draws the set Ramping over time  2 screen
static void LCDDrawSetOverTime2( void );             //add
///Callback funtion that draws the set temperature step  2 screen
static void LCDDrawSetTempStep2( void );    //chenglei add

static void LCDDrawSetFanSpeed1(void);
static void LCDDrawSetFanSpeed2(void);
static void LCDDrawFlowMeter1Optional(void);
static void LCDDrawFlowMeter2Optional(void);


///Callback function that handles input from buttons for the home screen
static void lcdcmd_home( int );
///Callback function that handles input from buttons for channel select screen
static void lcdcmd_groupsel( int );
///Callback function that handles input from buttons for the set target 1 screen
static void lcdcmd_settemp1( int  );
///Callback function that handles input from buttons for the set target 2 screen
static void lcdcmd_settemp2( int  );
///Callback function that handles input from buttons for the set threshold 1 screen
static void lcdcmd_setthresh1( int  );
///Callback function that handles input from buttons for the set threshold 2 screen
static void lcdcmd_setthresh2( int  );
///Callback function that handles input from buttons for the set mode 1 screen
static void lcdcmd_setmode1( int  );
///Callback function that handles input from buttons for the set mode 2 screen
static void lcdcmd_setmode2( int  );
///Callback function that handles input from buttons for the safety on/off 1 screen
static void lcdcmd_setsafeon1( int );
///Callback function that handles input from buttons for the safety on/off 2 screen
static void lcdcmd_setsafeon2( int );
///Callback function that handles input from buttons for the set safety time 1 screen
static void lcdcmd_setsafetime1( int );
///Callback function that handles input from buttons for the set safety time 2 screen
static void lcdcmd_setsafetime2( int );
///Callback function that handles input from buttons for the set safety degree 1 screen
static void lcdcmd_setsafedeg1( int );
///Callback function that handles input from buttons for the set safety degree 2 screen
static void lcdcmd_setsafedeg2( int );
///Callback function that handles input from buttons for the set PidMode
static void lcdcmd_setPidMode1( int );
///Callback function that handles input from buttons for the set P
static void lcdcmd_setP1( int );
///Callback function that handles input from buttons for the set I
static void lcdcmd_setI1( int );
///Callback function that handles input from buttons for the set D
static void lcdcmd_setD1( int );
///Callback function that handles input from buttons for the set PidMode
static void lcdcmd_setPidMode2( int );
///Callback function that handles input from buttons for the set P
static void lcdcmd_setP2( int );
///Callback function that handles input from buttons for the set I
static void lcdcmd_setI2( int );
///Callback function that handles input from buttons for the set D
static void lcdcmd_setD2( int );
///Callback function that handles input from buttons for the set max temperature  1 screen
static void lcdcmd_setMaxTemp1( int  );  
///Callback function that handles input from buttons for the set min temperature  1 screen
static void lcdcmd_setMinTemp1( int  );      
///Callback function that handles input from buttons for the set ramping coefficient  1 screen
static void lcdcmd_setcoefficient1( int  );                                              //add
///Callback function that handles input from buttons for the set ramping over time  1 screen
static void lcdcmd_setovertime1( int  );  
///Callback function that handles input from buttons for the set temp step  1 screen
static void lcdcmd_settempstep1( int  );    //chenglei add
///Callback function that handles input from buttons for the set max temperature  2 screen
static void lcdcmd_setMaxTemp2( int  );  
///Callback function that handles input from buttons for the set min temperature  2 screen
static void lcdcmd_setMinTemp2( int  );      
///Callback function that handles input from buttons for the set ramping coefficient 2 screen
static void lcdcmd_setcoefficient2( int  );                                         //add
///Callback function that handles input from buttons for the set ramping over time  2 screen
static void lcdcmd_setovertime2( int  ); 
///Callback function that handles input from buttons for the set temp step  2 screen
static void lcdcmd_settempstep2( int  );    //chenglei add

static void lcdcmd_fanspeed1( int ); 
static void lcdcmd_fanspeed2( int ); 
static void lcdcmd_FMOpt1(int up_down);
static void lcdcmd_FMOpt2(int up_down);
///Callback funtion that redraws the home screen
static void LCDReDrawHome( void );
///Callback funtion that redraws the Group Select screen
static void LCDReDrawGroupSelect( void ); 
///Callback funtion that redraws the set target 1 screen
static void LCDReDrawSetTgt1( void );
///Callback funtion that redraws the set threshold 1 screen
static void LCDReDrawSetThres1( void );
///Callback funtion that redraws the set mode 1 screen
static void LCDReDrawSetMode1( void );
///Callback funtion that redraws the set safety on/off 1 screen
static void LCDReDrawSetSafeOn1( void );
///Callback funtion that redraws the set safety timer 1 screen
static void LCDReDrawSetSafeTime1( void );
///Callback funtion that redraws the set safety degree 1 screen
static void LCDReDrawSetSafeDeg1( void );
///Callback funtion that redraws the set target 2 screen
static void LCDReDrawSetTgt2( void );
///Callback funtion that redraws the set thereshold 2 screen
static void LCDReDrawSetThres2( void );
///Callback funtion that redraws the set mode 2 screen
static void LCDReDrawSetMode2( void );
///Callback funtion that redraws the set safety on/off 2 screen
static void LCDReDrawSetSafeOn2( void );
///Callback funtion that redraws the set safety timer 2 screen
static void LCDReDrawSetSafeTime2( void );
///Callback funtion that redraws the set safety degree 2 screen
static void LCDReDrawSetSafeDeg2( void );
///Callback funtion that redraws the set PidMode
static void LCDReDrawSetPidMode1( void );
///Callback funtion that redraws the set P
static void LCDReDrawSetP1( void );
///Callback funtion that redraws the set I
static void LCDReDrawSetI1( void );
///Callback funtion that redraws the set D
static void LCDReDrawSetD1( void );
///Callback funtion that redraws the set PidMode
static void LCDReDrawSetPidMode2( void );
///Callback funtion that redraws the set P
static void LCDReDrawSetP2( void );
///Callback funtion that redraws the set I
static void LCDReDrawSetI2( void );
///Callback funtion that redraws the set D
static void LCDReDrawSetD2( void );
///Callback funtion that redraws the set max temperature 1 screen
static void LCDReDrawSetMaxTemp1( void );             //add
///Callback funtion that redraws the set min temperature 1 screen
static void LCDReDrawSetMinTemp1( void );             //add
///Callback funtion that redraws the set ramping coefficient 1 screen
static void LCDReDrawSetCoefficient1( void );             //add
///Callback funtion that redraws the set ramping over time 1 screen
static void LCDReDrawSetOverTime1( void );             //add
///Callback funtion that redraws the set temp step 1 screen
static void LCDReDrawSetTempStep1( void );             //chenglei add
///Callback funtion that redraws the set max temperature 1 screen
static void LCDReDrawSetMaxTemp2( void );             //add
///Callback funtion that redraws the set min temperature 2 screen
static void LCDReDrawSetMinTemp2( void );             //add
///Callback funtion that redraws the set ramping coefficient 1 screen
static void LCDReDrawSetCoefficient2( void );             //add
///Callback funtion that redraws the set ramping over time 2 screen
static void LCDReDrawSetOverTime2( void );             //add 
///Callback funtion that redraws the set temp step 2 screen
static void LCDReDrawSetTempStep2( void );             // chenglei add

static void LCDReDrawSetFanSpeed1( void );  
static void LCDReDrawSetFanSpeed2( void ); 

static void LCDReDrawFlowMeter1Optional();
static void LCDReDrawFlowMeter2Optional();
#define SAFEON 1
#define SAFEOFF 0

// Constants
#define UP				1
#define DOWN			(-1)

///Amount of time before a key will repeat (in ticks)
#define KEY_PRESS_DELAY  (1000 * portTICK_RATE_MS)
///Amount of time before a key repeats (after initial delay, in ticks)
#define KEY_REPEAT_DELAY (150  * portTICK_RATE_MS)

// Strings
//static const char* rs_curTemp1 	= "T1:";
//static const char* rs_curTemp2 	= "T2:";
//static const char* rs_tgtTemp1 	= "G1:";
//static const char* rs_tgtTemp2 	= "G2:";
//static const char* rs_curThre1 	= "H1:";
//static const char* rs_curThre2 	= "H2:";
static const char* rs_auto1 		= "A";
//static const char* rs_auto2 		= "A";
static const char* rs_pelt1 		= "P";
//static const char* rs_pelt2 		= "P";
static const char* rs_idle1	 	= "I";
//static const char* rs_idle2	 	= "I";
static const char* rs_fanon1	 	= "F";
//static const char* rs_fanon2	 	= "F";
static const char* rs_fan 		= "F]";
static const char* rs_heat	 	= "H]";
//static const char* rs_pelt    = "P]";
static const char* rs_clear		= "C]";
static const char* rs_cool = "C]";
static const char* rs_ramp1	= "R";    //add
static const char* rs_rampqc = "R";
//static const char* rs_open	 	= "[";
//static const char* rs_close	 	= "]";
//static const char* rs_on		= "o";
//static const char* rs_off	 	= "x";
//static const char* rs_next		= ">";

//static const char* pm_manual="MANU";
//static const char* pm_225w="225";
//static const char* pm_350w="350";
//static const char* pm_225fast="225F";
static const char* pm_225w_safety="225S";
static const char* pm_350w_safety="350S";
static const char* pm_na="N/A";

static char buf[64];
const char *rs_na = "N/A";

///Struct for containing the functions that a screen must perform
typedef struct _SLCDMenu {
	///function pointer to the draw function that draws the screen initially
    void (*Draw)(void);
    /**
    	function point to the function that will handle user input
    	\param up_down 1 == up, -1 == down
    */
    void (*function)(int up_down);
    ///function pointer to the redraw function that redraws the information onto the screen
    void (*ReDraw)(void);
} SLCDMenu;

#define LCDMENU_N 36

int MenuJmp = 0;
static SLCDMenu lcdMenu[LCDMENU_N] = {
    { LCDDrawHome, 	lcdcmd_home,          LCDReDrawHome},
    { LCDDrawGroupSelect,lcdcmd_groupsel, LCDReDrawGroupSelect },
    { LCDDrawSetTgt1, 	lcdcmd_settemp1,      LCDReDrawSetTgt1},
    { LCDDrawSetThres1,	lcdcmd_setthresh1,    LCDReDrawSetThres1},
    { LCDDrawSetMode1,	lcdcmd_setmode1,      LCDReDrawSetMode1},
  
    { LCDDrawSetMaxTemp1, lcdcmd_setMaxTemp1,  LCDReDrawSetMaxTemp1},            //add
    { LCDDrawSetMinTemp1, lcdcmd_setMinTemp1,  LCDReDrawSetMinTemp1},            //add
    { LCDDrawSetCoefficient1, lcdcmd_setcoefficient1,  LCDReDrawSetCoefficient1},            //add
    { LCDDrawSetOverTime1, lcdcmd_setovertime1,  LCDReDrawSetOverTime1}, 
    { LCDDrawSetTempStep1, lcdcmd_settempstep1,  LCDReDrawSetTempStep1},    // chenglei add 

    { LCDDrawSetSafeOn1,	lcdcmd_setsafeon1,    LCDReDrawSetSafeOn1},
    { LCDDrawSetSafeTime1,lcdcmd_setsafetime1,  LCDReDrawSetSafeTime1},
    { LCDDrawSetSafeDeg1,	lcdcmd_setsafedeg1,   LCDReDrawSetSafeDeg1},

	{ LCDDrawSetPidMode1,	lcdcmd_setPidMode1,   LCDReDrawSetPidMode1},
    { LCDDrawSetP1,	lcdcmd_setP1,   LCDReDrawSetP1},
    { LCDDrawSetI1,	lcdcmd_setI1,   LCDReDrawSetI1},
    { LCDDrawSetD1,	lcdcmd_setD1,   LCDReDrawSetD1},
    { LCDDrawSetFanSpeed1, lcdcmd_fanspeed1, LCDReDrawSetFanSpeed1 },
	{ LCDDrawFlowMeter1Optional,lcdcmd_FMOpt1,LCDReDrawFlowMeter1Optional}, 

    { LCDDrawSetTgt2, 	lcdcmd_settemp2,      LCDReDrawSetTgt2},
    { LCDDrawSetThres2,	lcdcmd_setthresh2,    LCDReDrawSetThres2},
    { LCDDrawSetMode2,	lcdcmd_setmode2,      LCDReDrawSetMode2},
   
    { LCDDrawSetMaxTemp2, lcdcmd_setMaxTemp2,  LCDReDrawSetMaxTemp2},            //add
    { LCDDrawSetMinTemp2, lcdcmd_setMinTemp2,  LCDReDrawSetMinTemp2},            //add
    { LCDDrawSetCoefficient2, lcdcmd_setcoefficient2,  LCDReDrawSetCoefficient2},            //add
    { LCDDrawSetOverTime2, lcdcmd_setovertime2,  LCDReDrawSetOverTime2}, 
    { LCDDrawSetTempStep2, lcdcmd_settempstep2,  LCDReDrawSetTempStep2},    // chenglei add 

    { LCDDrawSetSafeOn2,	lcdcmd_setsafeon2,    LCDReDrawSetSafeOn2},
    { LCDDrawSetSafeTime2,lcdcmd_setsafetime2,  LCDReDrawSetSafeTime2},
    { LCDDrawSetSafeDeg2,	lcdcmd_setsafedeg2,   LCDReDrawSetSafeDeg2},
    
	{ LCDDrawSetPidMode2,	lcdcmd_setPidMode2,   LCDReDrawSetPidMode2},
    { LCDDrawSetP2,	lcdcmd_setP2,   LCDReDrawSetP2},
    { LCDDrawSetI2,	lcdcmd_setI2,   LCDReDrawSetI2},
    { LCDDrawSetD2,	lcdcmd_setD2,   LCDReDrawSetD2} ,
    { LCDDrawSetFanSpeed2, lcdcmd_fanspeed2, LCDReDrawSetFanSpeed2 },
	{ LCDDrawFlowMeter2Optional,lcdcmd_FMOpt2,LCDReDrawFlowMeter2Optional}, 
};

///Current screen
static int lcdMenu_i = 0;

///flag to indicate if the screen must be completely redrawn
static unsigned int LCDFIX = 0;

// Button states
///state of up button
static int bs_u	= 0;
///state of down button
static int bs_d	= 0;
///state of left button
static int bs_l	= 0;
///state of right button
static int bs_r	= 0; 
static int bs_t = 5;
///last push time of up button
static portTickType bt_u = 0;
///last push time of down button
static portTickType bt_d = 0;
///last push time of left button
static portTickType bt_l = 0;
///last push time of right button
static portTickType bt_r = 0;
static portTickType bt_t = 0;
///Used to indicate if the information displayed on the LCD may have changed (doesn't do anything right now)
void LCD_info_changed(void) {
}

///Thread that handles the LCD interface
/**
	Thead that handles the LCD interface. This function will
	first wait for 1 sec (to let every other process start) then
	starts checking buttons/updating the display
 */
void vLCDInterface( void *pvParameters ) {
	unsigned int i;
	vTaskSetApplicationTaskTag(NULL, (void *)4);
	//wait for init to occur
	vTaskDelay( portTICK_RATE_MS * 500 );
	//check if the MAC is ff:ff:ff:ff:ff:ff, if so, skip displaying ethernet data
/*	if (!((cMACAddress[0] == 0xff)&&(cMACAddress[1] == 0xff)
			&&(cMACAddress[2] == 0xff)&&(cMACAddress[3] == 0xff)
			&&(cMACAddress[4] == 0xff)&&(cMACAddress[5] == 0xff))) {
		//delay for ethernet connection detect
		lcd_print(1, 0, "Waiting for ethernet", strlen("Waiting for ethernet"));
		lcd_print(2, 0, "Press button to skip", strlen("Press button to skip"));
		for (i = 0; i < 550; i++) {
			if (ethernetCableConnected || BUTTON_U || BUTTON_L || BUTTON_D || BUTTON_R || !ipIsEmpty())
				break;
			vTaskDelay( portTICK_RATE_MS * 10 );
		}
	
		lcd_print(1, 0, "                    ", strlen("                    "));
		lcd_print(2, 0, "                    ", strlen("                    "));

		//check if we are STATIC
		if (!ipIsEmpty()) {
			lcd_print(1, 0, "Static IP Address", strlen("Static IP Address"));
		}
		else if (ethernetCableConnected) { //DHCP
			lcd_print(1, 0, "Waiting for DHCP", strlen("Waiting for DHCP"));
			while (ipIsEmpty()) {
				if (BUTTON_U || BUTTON_D || BUTTON_L || BUTTON_R)
					break;
				lcdIPAddress();
			}
		}
		//display IP
		lcdIPAddress();
		lcd_print(3, 0, "Press button to skip", strlen("Press button to skip"));
		//wait so we don't over-write the IP information
		for (i = 0; i < 3000; i++) {
			//break on button press or no ethernet cable and DHCP
			if ((!ethernetCableConnected && ipIsEmpty()) || BUTTON_U || BUTTON_L || BUTTON_D || BUTTON_R)
				break;
			vTaskDelay( portTICK_RATE_MS * 10 );
		}
	}  */
    /* Loop forever */
    for( ;; ) {
        //we really should update the display every ~100ms or so, so it doesn't feel "chuggy"
        //we'll check the buttons every ~20ms (to ensure button presses are found)
        for (i = 0; i < 5; i++) {
			buttonIterate();
	
				vTaskDelay(portTICK_RATE_MS * 20);
				
        }
        displayIterate();
        
        
    }
}

/**
	Redraws the appropriate part of the display. Checks the variable
	LCDFIX

	\see LCDFIX
 */
void displayIterate( void ) {
    if (!LCDFIX) {

        lcdMenu[lcdMenu_i].Draw();
        LCDFIX = 1;
    } else {

        lcdMenu[lcdMenu_i].ReDraw();
    }
}

void buttonIterate( void ) {
    if ( risingEdge( BUTTON_L, &bs_l, NULL ) ) {
        if (lcdMenu_i == 0)
            lcdMenu_i = LCDMENU_N - 1;
        else
            lcdMenu_i--;
      
        LCDFIX = 0;
 
    }
    if ( risingEdge( BUTTON_D, &bs_d, &bt_d ) ) {
		if( lcdMenu_i != 1){
			if (bs_d == 2)
				lcdMenu[lcdMenu_i].function( DOWN*10 );
			else
				lcdMenu[lcdMenu_i].function( DOWN );
			//bs_ucount	= 0;
		}else{
				lcdMenu[lcdMenu_i].function( DOWN );
		}
    }

    if ( risingEdge( BUTTON_R, &bs_r, NULL ) ) {
		if (lcdMenu_i == 1) {
			if (MenuJmp != 0){
				lcdMenu_i = lcdMenu_i + 17; }	
			else{
				lcdMenu_i++;
			}
		}else{
			lcdMenu_i++;
			lcdMenu_i %= LCDMENU_N;
		}
        LCDFIX = 0;
    }

    if ( risingEdge( BUTTON_U, &bs_u, &bt_u ) ) {

		if( lcdMenu_i != 1){
			if (bs_u == 2)
				lcdMenu[lcdMenu_i].function( UP*10 );
			else
				lcdMenu[lcdMenu_i].function( UP );
			//bs_ucount	= 0;
		}else{
				lcdMenu[lcdMenu_i].function( UP );
		}
           
    }
}

/*//------------------------------------------------------------------------------*/
/// Check rising edge, also checks if the button has been held for more than KEY_PRESS_DELAY
/// then triggers every KEY_REPEAT_DELAY, which is in ms
///
//------------------------------------------------------------------------------
int risingEdge( int pin, int* last_state, portTickType *lastPush ) {
    int r	= 0;

    if ( ( *last_state == 0 ) && ( pin == 1 ) ) {
        r	= 1;
        if (lastPush != NULL)
	        *lastPush = xTaskGetTickCount();
        *last_state = pin;
    }
	else if ((lastPush != NULL) && (pin == 1) && (*last_state == 1) && (xTaskGetTickCount() > (*lastPush + KEY_PRESS_DELAY))) {
		*last_state = 2; //in repeat phase
		*lastPush = xTaskGetTickCount();
		r = 1;
	}
	else if ((lastPush != NULL) && (pin == 1) && (*last_state == 2) && (xTaskGetTickCount() > (*lastPush + KEY_REPEAT_DELAY))) {
		*lastPush = xTaskGetTickCount();
		r = 1;
	}
	else if(*last_state == 5)
   { 	
     r = 2;
   }
	if (pin == 0)
    	{
    		*last_state	= 0;
    	}

    return r ;
}


//------------------------------------------------------------------------------
///LCDDrawHome when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawHome() {
    lcd_print( 0, 0, "1:", 2);
    lcd_print( 0, 7, " ", 1);
    lcd_print( 0, 8, "2:", 2);
    lcd_print( 0, 15, " ", 1);
    lcd_print( 0, 16, "T1.7", 4);

    lcd_print( 1, 0, "T:", 2 );
    lcd_print( 1, 7, " ", 1);
    lcd_print( 1, 8, "T:", 2 );
    lcd_print( 1, 15, "    ", 5);

    lcd_print( 2, 0, "SET:", 4 );
    lcd_print( 2, 7, " ", 1);
    lcd_print( 2, 8, "SET:", 4 );
    lcd_print( 2, 15, " ", 1);
    lcd_print( 2, 16, "PRES", 4 );

    lcd_print( 3, 0, "                ", 16 );
    /*lcd_print( 3, 3, " ", 1);
    lcd_print( 3, 4, "S-", 2 );
    lcd_print( 3, 7, " ", 1);
    lcd_print( 3, 8, "M-", 2 );
    lcd_print( 3, 11, " ", 1);
    lcd_print( 3, 12, "S-", 2 );
    lcd_print( 3, 15, " ", 1);*/
    lcd_print( 3, 16, "BTN>", 4 );
}


static void LCDDrawGroupSelect(){
    lcd_print( 0, 0, "Group Config Select ", 20);
    lcd_print( 1, 0, "GROUP#1   <--     + ", 20);
    lcd_print( 2, 0, "GROUP#2          < >", 20);
    lcd_print( 3, 0, "                  - ", 20);

}
//------------------------------------------------------------------------------
///LCDDrawSetTgt1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetTgt1() {
    lcd_print( 0, 0, "SET TARGET TEMP     ", 20);

    lcd_print( 1, 0, "GROUP #1=", 9 );
    lcd_print( 1, 14, "    + ", 6);

    lcd_print( 2, 0, "ERROR            < >", 20);

    lcd_print( 3, 0, "THRESHOLD=", 10);
    lcd_print( 3, 11, "       ", 5);
    lcd_print( 3, 16, "  - ", 4);
}



//------------------------------------------------------------------------------
///LCDDrawSetTgt2 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetTgt2() {
    lcd_print( 0, 0, "SET TARGET TEMP     ", 20);

    lcd_print( 1, 0, "GROUP #2=", 9 );
    lcd_print( 1, 14, "    + ", 6);

    lcd_print( 2, 0, "ERROR            < >", 20);

    lcd_print( 3, 0, "THRESHOLD=", 10);
    lcd_print( 3, 11, "       ", 5);
    lcd_print( 3, 16, "  - ", 4);
}


//------------------------------------------------------------------------------
///LCDDrawSetThres1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetThres1() {
    lcd_print( 0, 0, "SET ERROR THRESHOLD ", 20);

    lcd_print( 1, 0, "GROUP #1=", 9 );
    lcd_print( 1, 14, "    + ", 6);

    lcd_print( 2, 0, "CURRENT          < >", 20);

    lcd_print( 3, 0, "TEMP=", 5);
    lcd_print( 3, 10, "        - ", 10);
}

//------------------------------------------------------------------------------
///LCDDrawSetThres2 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetThres2() {
    lcd_print( 0, 0, "SET ERROR THRESHOLD ", 20);

    lcd_print( 1, 0, "GROUP #2=", 9 );
    lcd_print( 1, 14, "    + ", 6);

    lcd_print( 2, 0, "CURRENT          < >", 20);

    lcd_print( 3, 0, "TEMP=", 5);
    lcd_print( 3, 10, "        - ", 10);
}

///Writes the string describing the current mode on the screen
void LCDDrawModeFullString(group *g) {
    int i = 0;

    i = get_group_mode(g);

    switch ( i ) {
    case MODE_AUTO:
        lcd_print( 2, 0, "Heater/fan used  ", 17 );
        lcd_print( 3, 0, "to reach target  ", 17 );
        break;
    case MODE_PELT:
        lcd_print( 2, 0, "Peltier used to  ", 17 );
        lcd_print( 3, 0, "heat and cool    ", 17 );
        break;
    case MODE_IDLE:
        lcd_print( 2, 0, "Heater/fan/pelt  ", 17 );
        lcd_print( 3, 0, "inactive         ", 17 );
        break;
    case MODE_FANON:
        lcd_print( 2, 0, "Fan on, heater   ", 17 );
        lcd_print( 3, 0, "and peltier off  ", 17 );
        break;
    case MODE_RAMP:
        lcd_print( 2, 0, "Ramp up & down   ", 17 );
        lcd_print( 3, 0, "between two temps", 17 );
        break;
    case MODE_RAMP_QC:
        lcd_print( 2, 0, "Ramp QC          ", 17 );
        lcd_print( 3, 0, "between two temps", 17 );
        break;
    default:
        lcd_print( 2, 0, "Unknown mode     ", 17 );
        lcd_print( 3, 0, "                 ", 17 );
    }
	
}

//------------------------------------------------------------------------------
///LCDDrawSetMode1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetMode1(  ) {
    lcd_print( 0, 0, "SET MODE            ", 20);

    lcd_print( 1, 0, "GROUP #1=", 9 );
    lcd_print( 1, 14, "    + ", 6);

    lcd_print( 2, 17, "< >", 3 );
    lcd_print( 3, 17, " + ", 3);

    LCDDrawModeFullString(S_GROUP1);
}


//------------------------------------------------------------------------------
///LCDDrawSetMode2 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetMode2(  ) {
    lcd_print( 0, 0, "SET MODE            ", 20);

    lcd_print( 1, 0, "GROUP #2=", 9 );
    lcd_print( 1, 14, "    + ", 6);

    lcd_print( 2, 17, "< >", 3);
    lcd_print( 3, 17, " + ", 3);

    LCDDrawModeFullString(S_GROUP2);
}
//------------------------------------------------------------------------------
///LCDDrawSetMaxTemp1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetMaxTemp1(  ) {
	lcd_print( 0, 0, "SET RAMP MAX TEMP   ", 20);

    lcd_print( 1, 0, "GROUP 1           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "MAX TEMP=", 9);
    lcd_print( 3, 13, "     - ", 7);
}
//------------------------------------------------------------------------------
///LCDDrawSetMinTemp1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetMinTemp1(  ) {
	lcd_print( 0, 0, "SET RAMP MIN TEMP   ", 20);

    lcd_print( 1, 0, "GROUP 1           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "MIN TEMP=", 9);
    lcd_print( 3, 13, "     - ", 7);
}
//------------------------------------------------------------------------------
///LCDDrawSetCoefficient1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetCoefficient1(  ) {
	  lcd_print( 0, 0, "SET COEFFICIENT     ", 20);

    lcd_print( 1, 0, "GROUP 1           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "COEFFICIENT=", 12);
    lcd_print( 3, 16, "  - ", 4);
}
//------------------------------------------------------------------------------
///LCDDrawSetOverTime1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetOverTime1(  ) {
	  lcd_print( 0, 0, "SET OVER TIME       ", 20);

    lcd_print( 1, 0, "RAMP TO NEXT TEMP + ", 20);

    lcd_print( 2, 0, "GROUP 1          < >", 20);

    lcd_print( 3, 0, "OVER TIME=", 10);
    lcd_print( 3, 16, "  - ", 4);
}
//------------------------------------------------------------------------------
///LCDDrawSetTempStep1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetTempStep1(  ) {
    lcd_print( 0, 0, "SET TEMP STEP     ", 20);

    lcd_print( 1, 0, "GROUP 1           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "STEP=", 5);
    lcd_print( 3, 9, "         - ", 11);
}
//------------------------------------------------------------------------------
///LCDDrawSetTempStep2 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetTempStep2(  ) {
    lcd_print( 0, 0, "SET TEMP STEP     ", 20);

    lcd_print( 1, 0, "GROUP 2           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "STEP=", 5);
    lcd_print( 3, 9, "         - ", 11);
}
//------------------------------------------------------------------------------
///LCDDrawSetMaxTemp1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetMaxTemp2(  ) {
	  lcd_print( 0, 0, "SET RAMP MAX TEMP   ", 20);

    lcd_print( 1, 0, "GROUP 2           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "MAX TEMP=", 9);
    lcd_print( 3, 13, "     - ", 7);
}
//------------------------------------------------------------------------------
///LCDDrawSetMinTemp1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetMinTemp2(  ) {
	  lcd_print( 0, 0, "SET RAMP MIN TEMP   ", 20);

    lcd_print( 1, 0, "GROUP 2           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "MIN TEMP=", 9);
    lcd_print( 3, 13, "     - ", 7);
}
//------------------------------------------------------------------------------
///LCDDrawSetCoefficient1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetCoefficient2(  ) {
	  lcd_print( 0, 0, "SET COEFFICIENT     ", 20);

    lcd_print( 1, 0, "GROUP 2           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "COEFFICIENT=", 12);
    lcd_print( 3, 16, "  - ", 4);
}
//------------------------------------------------------------------------------
///LCDDrawSetOverTime2 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetOverTime2(  ) {
	  lcd_print( 0, 0, "SET OVER TIME       ", 20);

    lcd_print( 1, 0, "RAMP TO NEXT TEMP + ", 20);

    lcd_print( 2, 0, "GROUP 2          < >", 20);

    lcd_print( 3, 0, "OVER TIME=", 10);
    lcd_print( 3, 16, "  - ", 4);
}
//------------------------------------------------------------------------------
///LCDDrawSetSafeOn1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetSafeOn1(  ) {
    lcd_print( 0, 0, "TURN SAFETY ON/OFF  ", 20);

    lcd_print( 1, 0, "GROUP #1=", 9 );
    lcd_print( 1, 12, "      + ", 8);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "                  - ", 20);
}

//------------------------------------------------------------------------------
///LCDDrawSetSafeOn2 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetSafeOn2(  ) {
    lcd_print( 0, 0, "TURN SAFETY ON/OFF  ", 20);

    lcd_print( 1, 0, "GROUP #2=", 9 );
    lcd_print( 1, 12, "      + ", 8);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "                  - ", 20);
}

//------------------------------------------------------------------------------
///LCDDrawSetSafeTime1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetSafeTime1(  ) {
    lcd_print( 0, 0, "NUMBER OF SECONDS   ", 20);

    lcd_print( 1, 0, "FOR X DEG RISE TO + ", 20);

    lcd_print( 2, 0, "AVOID SAFETY ERR < >", 20);

    lcd_print( 3, 0, "GROUP #1=", 9);
    lcd_print( 3, 12, "      - ", 8);
}


//------------------------------------------------------------------------------
///LCDDrawSetSafeTime2 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetSafeTime2(  ) {
    lcd_print( 0, 0, "NUMBER OF SECONDS   ", 20);

    lcd_print( 1, 0, "FOR X DEG RISE TO + ", 20);

    lcd_print( 2, 0, "AVOID SAFETY ERR < >", 20);

    lcd_print( 3, 0, "GROUP #2=", 9);
    lcd_print( 3, 12, "      - ", 8);
}


//------------------------------------------------------------------------------
///LCDDrawSetSafeTime1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetSafeDeg1(  ) {
    lcd_print( 0, 0, "NUMBER OF DEG RISE  ", 20);

    lcd_print( 1, 0, "IN X SECONDS TO   + ", 20);

    lcd_print( 2, 0, "AVOID SAFETY ERR < >", 20);

    lcd_print( 3, 0, "GROUP #1=", 9);
    lcd_print( 3, 12, "      - ", 8);
}


//------------------------------------------------------------------------------
///LCDDrawSetSafeTime2 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetSafeDeg2(  ) {
    lcd_print( 0, 0, "NUMBER OF DEG RISE  ", 20);

    lcd_print( 1, 0, "IN X SECONDS TO   + ", 20);

    lcd_print( 2, 0, "AVOID SAFETY ERR < >", 20);

    lcd_print( 3, 0, "GROUP #2=", 9);
    lcd_print( 3, 12, "      - ", 8);
}

//------------------------------------------------------------------------------
///LCDDrawSetPidMode1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetPidMode1(  ) {
    lcd_print( 0, 0, "PID MODE    ", 20);

    lcd_print( 1, 0, "GROUP 1           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "M=", 2);
    lcd_print( 3, 5, "             - ", 15);
}

//------------------------------------------------------------------------------
///LCDDrawSetP when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetP1(  ) {
    lcd_print( 0, 0, "PID PROPORTIONAL    ", 20);

    lcd_print( 1, 0, "GROUP 1           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "P=", 2);
    lcd_print( 3, 5, "             - ", 15);
}

//------------------------------------------------------------------------------
///LCDDrawSetI when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetI1(  ) {
    lcd_print( 0, 0, "PID INTEGRAL        ", 20);

    lcd_print( 1, 0, "GROUP 1           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "I=", 2);
    lcd_print( 3, 5, "             - ", 15);
}
//------------------------------------------------------------------------------
///LCDDrawSetI when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetD1(  ) {
    lcd_print( 0, 0, "PID DERIVATIVE      ", 20);

    lcd_print( 1, 0, "GROUP 1           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "D=", 2);
    lcd_print( 3, 5, "             - ", 15);
}
//------------------------------------------------------------------------------
///LCDDrawSetPidMode1 when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetPidMode2(  ) {
    lcd_print( 0, 0, "PID MODE    ", 20);

    lcd_print( 1, 0, "GROUP 2           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "M=", 2);
    lcd_print( 3, 5, "             - ", 15);
}

//------------------------------------------------------------------------------
///LCDDrawSetP when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetP2(  ) {
    lcd_print( 0, 0, "PID PROPORTIONAL    ", 20);

    lcd_print( 1, 0, "GROUP 2           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "P=", 2);
    lcd_print( 3, 5, "             - ", 15);
}

//------------------------------------------------------------------------------
///LCDDrawSetI when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetI2(  ) {
    lcd_print( 0, 0, "PID INTEGRAL        ", 20);

    lcd_print( 1, 0, "GROUP 2           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "I=", 2);
    lcd_print( 3, 5, "             - ", 15);
}
//------------------------------------------------------------------------------
///LCDDrawSetD when LCDFix is FALSE
//------------------------------------------------------------------------------
void LCDDrawSetD2(  ) {
    lcd_print( 0, 0, "PID INTEGRAL        ", 20);

    lcd_print( 1, 0, "GROUP 2           + ", 20);

    lcd_print( 2, 0, "                 < >", 20);

    lcd_print( 3, 0, "D=", 2);
    lcd_print( 3, 5, "             - ", 15);
}


void LCDDrawSetFanSpeed1() {
	
	lcd_print( 0, 0, "SET FAN THRESHOLD   ", 20);

    lcd_print( 1, 0, "GROUP #1=", 9 );
    lcd_print( 1, 14, "    + ", 6);

    lcd_print( 2, 0, "CURRENT          < >", 20);

    lcd_print( 3, 0, "THRES=", 6);
    lcd_print( 3, 10, "        - ", 10);
}

void LCDDrawSetFanSpeed2() {
	
	lcd_print( 0, 0, "SET FAN THRESHOLD   ", 20);

    lcd_print( 1, 0, "GROUP #2=", 9 );
    lcd_print( 1, 14, "    + ", 6);

    lcd_print( 2, 0, "CURRENT          < >", 20);

    lcd_print( 3, 0, "THRES=", 6);
    lcd_print( 3, 10, "        - ", 10);
}

void LCDDrawFlowMeter1Optional(){

	lcd_print( 0, 0, "FM1 Opt config      ", 20);
    lcd_print( 1, 0, "GROUP #1 ", 9 );
    lcd_print( 2, 0, "                 < >", 20);
    lcd_print( 3, 0, "                    ", 20);
}
void LCDDrawFlowMeter2Optional(){

	lcd_print( 0, 0, "FM2 Opt config      ", 20);
    lcd_print( 1, 0, "GROUP #2 ", 9 );
    lcd_print( 2, 0, "                 < >", 20);
    lcd_print( 3, 0, "                    ", 20);
}
//------------------------------------------------------------------------------
///NULL
//------------------------------------------------------------------------------
void lcdcmd_home(int i) {

}
static void lcdcmd_groupsel( int i){
	/*MenuJmp = MenuJmp + i;*/
	MenuJmp = (MenuJmp++)%2;
}
//------------------------------------------------------------------------------
///Set sensor 1 target temperature
//------------------------------------------------------------------------------
void lcdcmd_settemp1( int up_down ) {
    set_group_target(S_GROUP1, get_group_target_temp(S_GROUP1) + 25 * up_down);

}

//------------------------------------------------------------------------------
///Set sensor 2 target temperature
//------------------------------------------------------------------------------
void lcdcmd_settemp2( int up_down ) {
       set_group_target(S_GROUP2, get_group_target_temp(S_GROUP2) + 25 * up_down);
}

//------------------------------------------------------------------------------
///Set sensor 1 threshold temperature
//------------------------------------------------------------------------------
void lcdcmd_setthresh1( int up_down ) {
    set_group_threshold(S_GROUP1, get_group_threshold(S_GROUP1) + 25 * up_down);
}

//------------------------------------------------------------------------------
///Set sensor 2 threshold temperature
//------------------------------------------------------------------------------
void lcdcmd_setthresh2( int up_down ) {
    set_group_threshold(S_GROUP2, get_group_threshold(S_GROUP2) + 25 * up_down);
}


//------------------------------------------------------------------------------
///Set sensor 1 mode
//------------------------------------------------------------------------------
void lcdcmd_setmode1( int up_down ) {
    int i = 0;

    i = get_group_mode(S_GROUP1) + up_down;

    if ( i >= MODE_N ) {
        i = 0;
    } else if ( i < 0 ) {
        i = MODE_N - 1;
    }

    set_group_mode(S_GROUP1, i);
}

//------------------------------------------------------------------------------
///Set sensor 2 mode
//------------------------------------------------------------------------------
void lcdcmd_setmode2( int up_down ) {
    int i = 0;

    i = get_group_mode(S_GROUP2) + up_down;

    if ( i >= MODE_N ) {
        i = 0;
    } else if ( i < 0 ) {
        i = MODE_N - 1;
    }

    set_group_mode(S_GROUP2, i);
}

//------------------------------------------------------------------------------
///Set sensor 1 max temp
//------------------------------------------------------------------------------
void lcdcmd_setMaxTemp1( int up_down ) {
    set_group_max_temp( S_GROUP1, get_group_max_temp(S_GROUP1) + 25 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 2 max temp
//------------------------------------------------------------------------------
void lcdcmd_setMaxTemp2( int up_down ) {
    set_group_max_temp( S_GROUP2, get_group_max_temp(S_GROUP2) + 25 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 1 min temp
//------------------------------------------------------------------------------
void lcdcmd_setMinTemp1( int up_down ) {
    set_group_min_temp( S_GROUP1, get_group_min_temp(S_GROUP1) + 25 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 2 min temp
//------------------------------------------------------------------------------
void lcdcmd_setMinTemp2( int up_down ) {
    set_group_min_temp( S_GROUP2, get_group_min_temp(S_GROUP2) + 25 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 1 ramping coefficient
//------------------------------------------------------------------------------
void lcdcmd_setcoefficient1( int up_down ) {
    set_group_coefficient( S_GROUP1, get_group_coefficient(S_GROUP1) + 1000 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 2 ramping coefficient
//------------------------------------------------------------------------------
void lcdcmd_setcoefficient2( int up_down ) {
    set_group_coefficient( S_GROUP2, get_group_coefficient(S_GROUP2) + 1000 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 1 ramping over time
//------------------------------------------------------------------------------
void lcdcmd_setovertime1( int up_down ) {
    set_group_over_time( S_GROUP1, get_group_over_time(S_GROUP1) + 1000 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 2 ramping over time
//------------------------------------------------------------------------------
void lcdcmd_setovertime2( int up_down ) {
    set_group_over_time( S_GROUP2, get_group_over_time(S_GROUP2) + 1000 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 1 temp step
//------------------------------------------------------------------------------
void lcdcmd_settempstep1( int up_down ) {
    set_group_temp_step( S_GROUP1, get_group_temp_step(S_GROUP1) + 25 * up_down);
}
//------------------------------------------------------------------------------
///Set sensor 2 temp step
//------------------------------------------------------------------------------
void lcdcmd_settempstep2( int up_down ) {
    set_group_temp_step( S_GROUP2, get_group_temp_step(S_GROUP2) + 25 * up_down);
}
//------------------------------------------------------------------------------
///Set group 1 safe ON/OFF
//------------------------------------------------------------------------------
void lcdcmd_setsafeon1(int up_down) {
    if (get_group_safety(S_GROUP1))
        set_group_safety(S_GROUP1, 0); //disable timer
    else
        set_group_safety(S_GROUP1, SAFETY_TIMER_DEFAULT); //set to default 30 sec
}

//------------------------------------------------------------------------------
///Set group 2 safe ON/OFF
//------------------------------------------------------------------------------
void lcdcmd_setsafeon2(int up_down) {
    if (get_group_safety(S_GROUP2))
        set_group_safety(S_GROUP2, 0); //disable timer
    else
        set_group_safety(S_GROUP2, SAFETY_TIMER_DEFAULT); //set to default 30 sec
}

//------------------------------------------------------------------------------
///Set group 1 safe time
//------------------------------------------------------------------------------
void lcdcmd_setsafetime1(int up_down) {
    int i = 0;

    i = get_group_safety(S_GROUP1) + 1000 * up_down;

    if (i < 0)
        i = 0;
    else if (i > SAFETY_TIMER_MAX)
        i = SAFETY_TIMER_MAX;

    set_group_safety(S_GROUP1, i);
}

//------------------------------------------------------------------------------
///Set group 2 safe time
//------------------------------------------------------------------------------
void lcdcmd_setsafetime2(int up_down) {
    int i = 0;

    i = get_group_safety(S_GROUP2) + 1000 * up_down;

    if (i < 0)
        i = 0;
    else if (i > SAFETY_TIMER_MAX)
        i = SAFETY_TIMER_MAX;

    set_group_safety(S_GROUP2, i);
}


//------------------------------------------------------------------------------
///Set group 1 safe degree
//------------------------------------------------------------------------------
void lcdcmd_setsafedeg1(int up_down) {
    int i = 0;

    i = get_group_safety_degree(S_GROUP1) + up_down * 25;

    if (i < 0)
        i = 0;
    else if (i > SAFETY_DEGREE_MAX)
        i = SAFETY_DEGREE_MAX;

    set_group_safety_degree(S_GROUP1, i);
}


//------------------------------------------------------------------------------
///Set group 2 safe degree
//------------------------------------------------------------------------------
void lcdcmd_setsafedeg2(int up_down) {
    int i = 0;

    i = get_group_safety_degree(S_GROUP2) + up_down * 25;

    if (i < 0)
        i = 0;
    else if (i > SAFETY_DEGREE_MAX)
        i = SAFETY_DEGREE_MAX;

    set_group_safety_degree(S_GROUP2, i);
}

//------------------------------------------------------------------------------
///Set 
//------------------------------------------------------------------------------
void lcdcmd_setPidMode1(int up_down) {
    int i = 0;

    i = get_group_pid_mode(S_GROUP1) + up_down;
 /*   
    if (i<0)
        i = 6-1;
	else if (i>=6)
		i=0;
*/
    if (i<0)
        i = 2-1;
    else if (i>=2)
		i=0;
   
    set_group_pid_mode(S_GROUP1, i);
}

//------------------------------------------------------------------------------
///Set 
//------------------------------------------------------------------------------
void lcdcmd_setP1(int up_down) {
    int i = 0;
	
	if (get_group_pid_mode(S_GROUP1) ==  0)
	{
		i = get_group_pid_kp(S_GROUP1) + up_down*1;
		
		if (i<0)
			i = 0;

		set_group_pid_kp(S_GROUP1, i);
	}
}

//------------------------------------------------------------------------------
///Set 
//------------------------------------------------------------------------------
void lcdcmd_setI1(int up_down) {
    int i = 0;
	
	if (get_group_pid_mode(S_GROUP1) ==  0)
	{
		i = get_group_pid_ki(S_GROUP1) + up_down*1;
		
		if (i<0)
			i = 0;
			
		set_group_pid_ki(S_GROUP1, i);
	}
}
//------------------------------------------------------------------------------
///Set 
//------------------------------------------------------------------------------
void lcdcmd_setD1(int up_down) {
    int i = 0;
	
	if (get_group_pid_mode(S_GROUP1) ==  0)
	{
		i = get_group_pid_kd(S_GROUP1) + up_down*1;
		
		if (i<0)
			i = 0;
			
		set_group_pid_kd(S_GROUP1, i);
	}
}
//------------------------------------------------------------------------------
///Set 
//------------------------------------------------------------------------------
void lcdcmd_setPidMode2(int up_down) {
    int i = 0;

    i = get_group_pid_mode(S_GROUP2) + up_down;
    
 /*   if (i<0)
        i = 6-1;
	else if (i>=6)
		i=0;
*/
   if (i<0)
        i = 2-1;
	else if (i>=2)
		i=0;
    set_group_pid_mode(S_GROUP2, i);
}

//------------------------------------------------------------------------------
///Set 
//------------------------------------------------------------------------------
void lcdcmd_setP2(int up_down) {
    int i = 0;
	
	if (get_group_pid_mode(S_GROUP2) ==  0)
	{
		i = get_group_pid_kp(S_GROUP2) + up_down*1;
		
		if (i<0)
			i = 0;

		set_group_pid_kp(S_GROUP2, i);
	}
}

//------------------------------------------------------------------------------
///Set 
//------------------------------------------------------------------------------
void lcdcmd_setI2(int up_down) {
    int i = 0;
	
	if (get_group_pid_mode(S_GROUP2) ==  0)
	{
		i = get_group_pid_ki(S_GROUP2) + up_down*1;
		
		if (i<0)
			i = 0;
			
		set_group_pid_ki(S_GROUP2, i);
	}
}
//------------------------------------------------------------------------------
///Set 
//------------------------------------------------------------------------------
void lcdcmd_setD2(int up_down) {
    int i = 0;
	
	if (get_group_pid_mode(S_GROUP2) ==  0)
	{
		i = get_group_pid_kd(S_GROUP2) + up_down*1;
		
		if (i<0)
			i = 0;
			
		set_group_pid_kd(S_GROUP2, i);
	}
}

void lcdcmd_fanspeed1( int up_down) { 
	
	 set_group_threshold_fan(S_GROUP1, get_group_threshold_fan(S_GROUP1) + 10 * up_down);
}

void lcdcmd_fanspeed2( int up_down) { 
	
	 set_group_threshold_fan(S_GROUP2, get_group_threshold_fan(S_GROUP2) + 10 * up_down);
}
void lcdcmd_FMOpt1(int up_down){
	if (flowMeterOptional_get(1))
	  flowMeterOptional_clear(1);
	else 
	  flowMeterOptional_set(1);

}
void lcdcmd_FMOpt2(int up_down){
	if (flowMeterOptional_get(2))
	  flowMeterOptional_clear(2);
	else 
	  flowMeterOptional_set(2);
}
//------------------------------------------------------------------------------
///LCDReDrawHome when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawHome() {
    int i = 0;
    int j= 0;
    lcd_print( 0, 2, get_sensor_short_name(S_GROUP1), 5 );
    lcd_print( 0, 10, get_sensor_short_name(S_GROUP2), 5 );

    lcd_print( 1, 2, formatTemp( get_sensor_temp(S_GROUP1), buf ), 5 );
    lcd_print( 1, 10, formatTemp( get_sensor_temp(S_GROUP2), buf ), 5 );
//    lcd_print (1, 17, itoa(count ,buf), 2 );
    lcd_print( 2, 4, itoa( get_group_target_temp(S_GROUP1) / 100, buf ), 3 );
    lcd_print( 2, 12, itoa( get_group_target_temp(S_GROUP2) / 100, buf ), 3 );
    
    //-----------Display PWM info---------------
  //  lcd_print( 3, 0, itoa(-1*get_group_pwm(S_GROUP1),buf), 4 );
  //  lcd_print( 3, 8, itoa(-1*get_group_pwm(S_GROUP2),buf), 4 );  
   //------------Display PID Mode info--------------------------
     j= get_group_pid_mode(S_GROUP1);
    switch ( j ) {
 /*   case 0:
        lcd_print( 3, 0, pm_manual, 4 );
        break;
    case 1:
        lcd_print( 3, 0, pm_225w, 4 );
        break;
    case 2:
        lcd_print( 3, 0, pm_350w, 4 );
        break;
	  case 3:
        lcd_print( 3, 0, pm_225fast, 4 );
        break;
    case 4:
    	  lcd_print( 3, 0, pm_225w_safety, 4 );
        break;
    case 5:
    	  lcd_print( 3, 0, pm_350w_safety, 4 );
        break;  */
      case 0:
      	    lcd_print( 3, 0, pm_225w_safety, 4 );
            break;
       case 1:
       	    lcd_print( 3, 0, pm_350w_safety, 4 );
            break;
    default:
        lcd_print( 3, 0, pm_na, 4);
  }  
    //-----------Display Group1 info------------
    i = get_group_mode(S_GROUP1);

    switch ( i ) {
    case MODE_AUTO:
        lcd_print( 3, 5, rs_auto1, 1 );
        break;
    case MODE_PELT:
        lcd_print( 3, 5, rs_pelt1, 1 );
        break;
    case MODE_IDLE:
        lcd_print( 3, 5, rs_idle1, 1 );
        break;
    case MODE_FANON:
        lcd_print( 3, 5, rs_fanon1, 1 );
        break;
    case MODE_RAMP:
    	lcd_print( 3, 5, rs_ramp1, 1 );
        break;
    case MODE_RAMP_QC:
        lcd_print( 3, 5, rs_rampqc, 1 );
        break;  
    default:
        lcd_print( 3, 5, rs_na, 1 );
    }

    if (get_heat_driver_mode(S_GROUP1) != DRIVER_MODE_OFF) {
        if (i == MODE_PELT || i== MODE_RAMP) {
            if (get_heat_driver_mode(S_GROUP1) == DRIVER_MODE_COOL) {
                lcd_print( 3, 6, rs_cool, 1 );
            } else {
                lcd_print( 3, 6, rs_heat, 1 );
            }
        } 
        else if (i == MODE_RAMP_QC){
            if (get_heat_driver_mode(S_GROUP1) == DRIVER_MODE_COOL) {
                lcd_print( 3, 6, rs_fan, 1 );
            } else {
                lcd_print( 3, 6, rs_heat, 1 );
            }
        }            
        else {
            lcd_print( 3, 6, rs_heat, 1 );
        }
    } else if (get_fan_driver_mode(S_GROUP1) != DRIVER_MODE_OFF) {
        lcd_print( 3, 6, rs_fan, 1 );
    } else {
        lcd_print( 3, 6, rs_clear, 1 );
    }
     //-------------Display PID Mode info----------
     j= get_group_pid_mode(S_GROUP2);
    switch ( j ) {
   /* case 0:
        lcd_print( 3, 8, pm_manual, 4 );
        break;
    case 1:
        lcd_print( 3, 8, pm_225w, 4 );
        break;
    case 2:
        lcd_print( 3, 8, pm_350w, 4 );
        break;
	  case 3:
        lcd_print( 3, 8, pm_225fast, 4 );
        break;
     case 4:
    	  lcd_print( 3, 8, pm_225w_safety, 4 );
        break;
    case 5:
    	  lcd_print( 3, 8, pm_350w_safety, 4 );
        break; */
     case 0:
    	  lcd_print( 3, 8, pm_225w_safety, 4 );
        break;
    case 1:
    	  lcd_print( 3, 8, pm_350w_safety, 4 );
        break;  
    default:
        lcd_print( 3, 8, pm_na, 4);
        
      }  
    //-----------Display Group2 info------------
    i = get_group_mode(S_GROUP2);

    switch ( i ) {
    case MODE_AUTO:
        lcd_print( 3, 13, rs_auto1, 1 );
        break;
    case MODE_PELT:
        lcd_print( 3, 13, rs_pelt1, 1 );
        break;
    case MODE_IDLE:
        lcd_print( 3, 13, rs_idle1, 1 );
        break;
    case MODE_FANON:
        lcd_print( 3, 13, rs_fanon1, 1 );
        break;
    case MODE_RAMP:
    	lcd_print( 3, 13, rs_ramp1, 1 );
    case MODE_RAMP_QC:
        lcd_print( 3, 13, rs_rampqc, 1 );
        break;
    default:
        lcd_print( 3, 13, rs_na, 1 );
    }

    if (get_heat_driver_mode(S_GROUP2) != DRIVER_MODE_OFF) {
        if (i == MODE_PELT|| i== MODE_RAMP) {
            if (get_heat_driver_mode(S_GROUP2) == DRIVER_MODE_COOL) {
                lcd_print( 3, 14, rs_cool, 1 );
            }
            else{
                lcd_print( 3, 14, rs_heat, 1 );   
            }
        }
        else if (i == MODE_RAMP_QC){
            if (get_heat_driver_mode(S_GROUP1) == DRIVER_MODE_COOL) {
                lcd_print( 3, 14, rs_fan, 1 );
            } else {
                lcd_print( 3, 14, rs_heat, 1 );
            }
        }
        else {
            lcd_print( 3, 14, rs_heat, 1 );
        }
    } else if (get_fan_driver_mode(S_GROUP2) != DRIVER_MODE_OFF) {
        lcd_print( 3, 14, rs_fan, 1 );
    } else {
        lcd_print( 3, 14, rs_clear, 1 );
    }
}
void LCDReDrawGroupSelect() {
  if ( 0 == MenuJmp ){
    lcd_print( 1, 0, "GROUP#1   <--     + ", 20);
    lcd_print( 2, 0, "GROUP#2          < >", 20);
  }else{
    lcd_print( 1, 0, "GROUP#1           + ", 20);
    lcd_print( 2, 0, "GROUP#2   <--    < >", 20);
  }
}	


//------------------------------------------------------------------------------
///LCDReDrawSetTgt1 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetTgt1() {
    lcd_print( 1, 9, formatTemp( get_group_target_temp(S_GROUP1), buf ), 5 );
    lcd_print( 3, 10, formatTemp( get_group_threshold(S_GROUP1), buf ), 5 );
}


//------------------------------------------------------------------------------
///LCDReDrawSetTgt2 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetTgt2() {
    lcd_print( 1, 9, formatTemp( get_group_target_temp(S_GROUP2), buf ), 5 );
    lcd_print( 3, 10, formatTemp( get_group_threshold(S_GROUP2), buf ), 5 );
}


//------------------------------------------------------------------------------
///LCDReDrawSetThres1 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetThres1() {
    lcd_print( 1, 9, formatTemp( get_group_threshold(S_GROUP1), buf ), 5 );
    lcd_print( 3, 5, formatTemp( get_sensor_temp(S_GROUP1), buf ), 5);
}

//------------------------------------------------------------------------------
///LCDReDrawSetThres2 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetThres2() {
    lcd_print( 1, 9, formatTemp( get_group_threshold(S_GROUP2), buf ), 5 );
    lcd_print( 3, 5, formatTemp( get_sensor_temp(S_GROUP2), buf ), 5);
}


//------------------------------------------------------------------------------
///LCDReDrawSetMode1 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetMode1() {
    int i = 0;

    i = get_group_mode(S_GROUP1);

    switch ( i ) {
    case MODE_AUTO:
        lcd_print( 1, 9, "AUTO ", 5 );
        break;
    case MODE_PELT:
        lcd_print( 1, 9, "PELT ", 5 );
        break;
    case MODE_IDLE:
        lcd_print( 1, 9, "IDLE ", 5 );
        break;
    case MODE_FANON:
        lcd_print( 1, 9, "FANON", 5 );
        break;
    case MODE_RAMP:
    	lcd_print( 1, 9, "RAMP ", 5 );
        break;
    case MODE_RAMP_QC:
        lcd_print( 1, 9, "RAMPQ", 5 );
        break;
    default:
        lcd_print( 1, 9, "N/A  ", 5 );
    }
    LCDDrawModeFullString(S_GROUP1);
}


//------------------------------------------------------------------------------
///LCDReDrawSetMode2 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetMode2() {
    int i = 0;

    i = get_group_mode(S_GROUP2);

    switch ( i ) {
    case MODE_AUTO:
        lcd_print( 1, 9, "AUTO ", 5 );
        break;
    case MODE_PELT:
        lcd_print( 1, 9, "PELT ", 5 );
        break;
    case MODE_IDLE:
        lcd_print( 1, 9, "IDLE ", 5 );
        break;
    case MODE_FANON:
        lcd_print( 1, 9, "FANON", 5 );
        break;
    case MODE_RAMP:
    	  lcd_print( 1, 9, "RAMP", 5 );
        break;
    case MODE_RAMP_QC:
        lcd_print( 1, 9, "RAMPQ", 5 );
        break;
    default:
        lcd_print( 1, 9, "N/A  ", 5 );
    }
    LCDDrawModeFullString(S_GROUP2);
}

//------------------------------------------------------------------------------
///LCDReDrawSetMaxTemp when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetMaxTemp1() {
    lcd_print( 3, 9, formatTemp(get_group_max_temp(S_GROUP1), buf ), 4 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetMinTemp when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetMinTemp1() {
    lcd_print( 3, 9, formatTemp(get_group_min_temp(S_GROUP1), buf ), 4 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetCoefficient when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetCoefficient1() {
    lcd_print( 3, 12, itoa(get_group_coefficient(S_GROUP1)/1000, buf ), 4 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetOverTime when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetOverTime1() {
    lcd_print( 3, 10, itoa(get_group_over_time(S_GROUP1)/1000, buf ), 5 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetTempStep when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetTempStep1() {
    lcd_print( 3, 5, formatTemp(get_group_temp_step(S_GROUP1), buf ), 4 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetTempStep when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetTempStep2() {
    lcd_print( 3, 5, formatTemp(get_group_temp_step(S_GROUP2), buf ), 4 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetMaxTemp when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetMaxTemp2() {
    lcd_print( 3, 9, formatTemp(get_group_max_temp(S_GROUP2), buf ), 4 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetMinTemp when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetMinTemp2() {
    lcd_print( 3, 9, formatTemp(get_group_min_temp(S_GROUP2), buf ), 4 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetCoefficient when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetCoefficient2() {
    lcd_print( 3, 12, itoa(get_group_coefficient(S_GROUP2)/1000, buf ), 4 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetOverTime when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetOverTime2() {
    lcd_print( 3, 10, itoa(get_group_over_time(S_GROUP2)/1000, buf ), 5 );   
}
//------------------------------------------------------------------------------
///LCDReDrawSetSafeOn1 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetSafeOn1() {
    if (get_group_safety(S_GROUP1) != 0)
        lcd_print( 1, 9, "ON ", 3 );
    else
        lcd_print( 1, 9, "OFF", 3 );

}

//------------------------------------------------------------------------------
///LCDReDrawSetSafeOn2 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetSafeOn2() {
    if (get_group_safety(S_GROUP2) != 0)
        lcd_print( 1, 9, "ON ", 3 );
    else
        lcd_print( 1, 9, "OFF", 3 );

}


//------------------------------------------------------------------------------
///LCDReDrawSetSafeTime1 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetSafeTime1() {
    lcd_print( 3, 9, itoa( get_group_safety(S_GROUP1)/1000, buf ), 3 );
}

//------------------------------------------------------------------------------
///LCDReDrawSetSafeTime2 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetSafeTime2() {
    lcd_print( 3, 9, itoa( get_group_safety(S_GROUP2)/1000, buf ), 3 );
}

//------------------------------------------------------------------------------
///LCDReDrawSetSafeDeg1 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetSafeDeg1() {
    lcd_print( 3, 9, formatTemp( get_group_safety_degree(S_GROUP1), buf ), 5 );
}


//------------------------------------------------------------------------------
///LCDReDrawSetSafeDeg2 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetSafeDeg2() {
    lcd_print( 3, 9, formatTemp( get_group_safety_degree(S_GROUP2), buf ), 5 );
}

//------------------------------------------------------------------------------
///LCDReDrawSetPidMode1 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetPidMode1() {
	int i = 0;

  i = get_group_pid_mode(S_GROUP1);

    switch ( i ) {
/*    case 0:
        lcd_print( 3, 2, "MANUAL    ", 10 );
        break;
    case 1:
        lcd_print( 3, 2, "P225W     ", 10 );
        break;
    case 2:
        lcd_print( 3, 2, "P350W     ", 10 );
        break;
	case 3:
        lcd_print( 3, 2, "P225W FAST", 10 );
        break;
   case 4:
        lcd_print( 3, 2, "225W SAFTY", 10 );
        break;
    case 5:
        lcd_print( 3, 2, "350W SAFTY", 10 );
        break;   */
    case 0:
        lcd_print( 3, 2, "225W SAFTY", 10 );
        break;
    case 1:
        lcd_print( 3, 2, "350W SAFTY", 10 );
        break; 
    default:
        lcd_print( 3, 2, "N/A       ", 10);
    }
}

//------------------------------------------------------------------------------
///LCDReDrawSetP when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetP1() {
    lcd_print( 3, 2, formatTemp(get_group_pid_kp(S_GROUP1) , buf ), 5 );
}

//------------------------------------------------------------------------------
///LCDReDrawSetP when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetI1() {
    lcd_print( 3, 2, formatTemp(get_group_pid_ki(S_GROUP1) , buf ), 5 );
}
//------------------------------------------------------------------------------
///LCDReDrawSetD when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetD1() {
    lcd_print( 3, 2, formatTemp(get_group_pid_kd(S_GROUP1) , buf ), 5 );
}
//------------------------------------------------------------------------------
///LCDReDrawSetPidMode1 when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetPidMode2() {
    int i = 0;

    i = get_group_pid_mode(S_GROUP2);

    switch ( i ) {
/*    case 0:
        lcd_print( 3, 2, "MANUAL    ", 10 );
        break;
    case 1:
        lcd_print( 3, 2, "P225W     ", 10 );
        break;
    case 2:
        lcd_print( 3, 2, "P350W     ", 10 );
        break;
   	case 3:
        lcd_print( 3, 2, "P225W FAST", 10 );
        break;
    case 4:
        lcd_print( 3, 2, "225W SAFTY", 10 );
        break;
    case 5:
        lcd_print( 3, 2, "350W SAFTY", 10 );
        break;   */
     case 0:
        lcd_print( 3, 2, "225W SAFTY", 10 );
        break;
    case 1:
        lcd_print( 3, 2, "350W SAFTY", 10 );
        break;  
    default:
        lcd_print( 3, 2, "N/A       ", 10);
    }
}

//------------------------------------------------------------------------------
///LCDReDrawSetP when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetP2() {
    lcd_print( 3, 2, formatTemp(get_group_pid_kp(S_GROUP2) , buf ), 5 );
}

//------------------------------------------------------------------------------
///LCDReDrawSetP when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetI2() {
    lcd_print( 3, 2, formatTemp(get_group_pid_ki(S_GROUP2) , buf ), 5 );
}
//------------------------------------------------------------------------------
///LCDReDrawSetP when LCDFix is TURE
//------------------------------------------------------------------------------
void LCDReDrawSetD2() {
    lcd_print( 3, 2, formatTemp(get_group_pid_kd(S_GROUP2) , buf ), 5 );
}

void uilcd_flashErr(unsigned int onZero) {
    if ( onZero == 0 ) {
        if (((AT91C_BASE_PIOB->PIO_ODSR & 0x00400000) >> 22))
            AT91C_BASE_PIOB->PIO_CODR	= (1 << 22);
        else
            AT91C_BASE_PIOB->PIO_SODR	= (1 << 22);
    }
}

void LCDReDrawSetFanSpeed1() {
 
    lcd_print( 1, 9, itoa( get_group_fan_speed(S_GROUP1), buf ), 5 );
    lcd_print( 3, 6, itoa( get_group_threshold_fan(S_GROUP1), buf ), 5 );
}
void LCDReDrawSetFanSpeed2() {
 
   lcd_print( 1, 9, itoa( get_group_fan_speed(S_GROUP2), buf ), 5 );
   lcd_print( 3, 6, itoa( get_group_threshold_fan(S_GROUP2), buf ), 5 );
}
void LCDReDrawFlowMeter1Optional(){

	if (flowMeterOptional_get(1))
	  lcd_print( 1, 9, "SET    ", 7 );
	else 
	  lcd_print( 1, 9, "CLRED  ", 7 );
}
void LCDReDrawFlowMeter2Optional(){

	if (flowMeterOptional_get(2))
	  lcd_print( 1, 9, "SET    ", 7 );
	else 
	  lcd_print( 1, 9, "CLRED  ", 7 );
}
