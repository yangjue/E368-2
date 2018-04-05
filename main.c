/*
    FreeRTOS V6.0.5 - Copyright (C) 2010 Real Time Engineers Ltd.
 
    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS eBook                                  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/
/** \mainpage E368 Code Documentation
	\section intro Introduction
	This is the documentation for the E368 Atmel running FreeRTOS. This documentation
	describes the software interfaces that have been built up to make the application
	run properly while still being maintainable.
	\section org Code Organization
	The code is mostly split into to a few main groups, drivers, interfaces, and sensors. 
	The drivers	are generic interfaces to the hardware (say SPI, or I2C). Interfaces are 
	things that the user sees such as LEDs, LCD, serial. The sensors and heaters are
	put into a single group that interfaces to all of the different possible sensors
	and heaters/coolers. Finally there is the control and safety code which runs on top
	of this large framework.

	The advantage to this style of organization is that the control code is very simple,
	so is the safety code. All of the details have been abstracted away. 
	\subsection drivers Drivers
	The different drivers implemented can be found under the e368/drivers folder. These include:
	UART, I2C, SPI, LCD, TCP, PWM, and USB.
	\subsection interfaces Interfaces
	The interfaces can be found in e368/ folder, the main ones are the terminal and lcdInterface.
	The terminal runs over any of the serial interfaces (RS232, USB, or TCP), and the lcdInterface
	run (of course) on the LCD.
	interfaces
	\subsection sensors	Sensors and Drivers
	The sensors and drivers are found in the sensors.c file. This is really a group of 
	functions and a data structure that organizes and defines the method of gathering
	a temperature and controlling the applied temperature.
	\section control_safety Control and Safety
	The control code can be found in control.c and the safety code can be found in 
	safety.c. These routines are simple and are run in different threads for a reason.
	The safety thread is the most important thread because it keeps the system from
	burning down, thus it runs by itself and at a higher priority.
	
*/

/// \file main.c main file

/*
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the application tasks, then starts the scheduler.
 *
 */

/* Library includes. */
#include <string.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* application includes. */
//#include "Drivers/TCP/tcpSerial.h"
#include "Drivers/USB/USB-CDC.h"
#include "Drivers/UART/uartDriver.h"
#include "Drivers/LCD/lcdDriver.h"
#include "Drivers/SPI/spiDriver.h"
#include "Drivers/PWM/pwmDriver.h"
#include "Drivers/I2C/i2cDriver.h"
//#include "Drivers/EMAC/SAM7_EMAC.h"

/* lwIP includes. */
//#include "lwip/api.h"

/* Hardware specific headers. */
#include "Board.h"
#include "AT91SAM7X256.h"
#include "pins.h"
#include "terminal.h"
#include "lcdInterface.h"
#include "control.h"
#include "safety.h"
#include "adc.h"
#include "Drivers/eeprom.h"

/* Priorities/stacks for the various tasks within the demo application. */
//#define mainTCP_PRIORITY      		( tskIDLE_PRIORITY + 3 )
#define mainSERIAL_UI_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainLCD_UI_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainCONTROL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainSAFETY_PRIORITY			( tskIDLE_PRIORITY + 3 )
#define mainPOLL_PRIORITY		     ( tskIDLE_PRIORITY + 2 )
//drivers
#define mainUSB_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainUART_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainLCD_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainSPI_PRIORITY			( tskIDLE_PRIORITY + 2 )

#define mainUSB_TASK_STACK			( 200 )
#define mainUART_TASK_STACK			( 200 )
#define mainLCD_TASK_STACK			( 200 )
#define mainSPI_TASK_STACK			( 200 )
#define mainSERIAL_UI_TASK_STACK	( 400 )
#define mainLCD_UI_TASK_STACK		( 200 )
#define mainCONTROL_STACK			( 200 )
#define mainSAFETY_STACK			( 200 )
#define mainPOLL_STACK			( 200 )
/*-----------------------------------------------------------*/
static void prvSetupHardware( void );
void vApplicationIdleHook( void );
/*-----------------------------------------------------------*/

unsigned char boardId[8];

/**
 * Setup hardware then start all the application tasks.
 */
int main( void ) {
    /* Setup the ports. */
    prvSetupHardware();
	
	//load id from eeprom
 getIdInit(boardId);
	//small delay
	Delay100NCycles(100);
	//load mac from eeprom
//	getMacInit(cMACAddress);
	//small delay
	Delay100NCycles(100);
	//load ip from eeprom
//	getIpInit(ucIPAddress);

    /* Setup lwIP. */
 //   vlwIPInit();

    /* Create the lwIP (ethernet) task.  This uses the lwIP RTOS abstraction layer.*/
//   sys_thread_new( vBasicWEBServer, ( void * ) NULL, mainTCP_PRIORITY );

    /* Create the USB CDC task. */
   xTaskCreate( vUSBCDCTask, (signed char *)	 	"USB_Driver", 	mainUSB_TASK_STACK, 		NULL, mainUSB_PRIORITY, 		NULL );

   /* Create the UART Interface task. */
   xTaskCreate( vUartDriver, (signed char *) 		"Uart_Driver", 	mainUART_TASK_STACK, 		NULL, mainUART_PRIORITY, 		NULL );

   /* Create the LCD Driver task */
  	xTaskCreate( vLCDDriver, (signed char *) 		"LCD_Driver", 	mainLCD_TASK_STACK, 		NULL, mainLCD_PRIORITY, 		NULL );

   /* Create the SPI Driver task */
   xTaskCreate( vSPIDriver, (signed char *) 		"SPI_Driver", 	mainSPI_TASK_STACK, 		NULL, mainSPI_PRIORITY, 		NULL);

   /* Create the serial/usb/ether interface */
   xTaskCreate( vTerminalTask, (signed char *) 	"Terminal", 	mainSERIAL_UI_TASK_STACK,	NULL, mainSERIAL_UI_PRIORITY, 	NULL);

/* Create the LCD interface */
xTaskCreate( vLCDInterface, (signed char *)		"LCD Interface", mainLCD_UI_TASK_STACK,		NULL, mainLCD_UI_PRIORITY,		NULL);

///* Create the control task */
xTaskCreate( vControlTask, (signed char*)		"Control Task", mainCONTROL_STACK,			NULL, mainCONTROL_PRIORITY,		NULL);
//
/* Create the safety task */
xTaskCreate( vSafetyTask, (signed char *)		"Safety Task",	mainSAFETY_STACK,			NULL, mainSAFETY_PRIORITY,		NULL);   
	/* Create the polling task */
	xTaskCreate( vPollingTask, (signed char*)		"Polling Task", mainPOLL_STACK,			NULL, mainPOLL_PRIORITY,		NULL);
	
    //turn off all the LEDs
    LED_ERROR_OFF;
    LED_STABLE_OFF;
    LED_HEATON_OFF;
    LED_FANON_OFF;

    /* Finally, start the scheduler.

    /* Finally, start the scheduler.

    /* Finally, start the scheduler.

    /* Finally, start the scheduler.

    /* Finally, start the scheduler.

    NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
    The processor MUST be in supervisor mode when vTaskStartScheduler is
    called.  The demo applications included in the FreeRTOS.org download switch
    to supervisor mode prior to main being called.  If you are not using one of
    these demo application projects then ensure Supervisor mode is used here. */

    
// AT91C_BASE_PIOA->PIO_PPUER = AT91C_BASE_PIOA->PIO_PPUER |0x12000000;   
// AT91C_BASE_PIOA->PIO_OER  =  AT91C_BASE_PIOA->PIO_OER |  0x02000000;             
// AT91C_BASE_PIOA->PIO_ODR = AT91C_BASE_PIOA->PIO_ODR & 0xFDFFFFFF;  
 
 vTaskStartScheduler(); 

    /* Should never get here! */
    return 0;
}
/*-----------------------------------------------------------*/

/**
	Setup all the hardware
 */
static void prvSetupHardware( void ) {
    /* When using the JTAG debugger the hardware is not always initialised to
    the correct default state.  This line just ensures that this does not
    cause all interrupts to be masked at the start. */
    AT91C_BASE_AIC->AIC_EOICR = 0;

    /* Most setup is performed by the low level init function called from the
    startup asm file.

    Configure the PIO Lines corresponding to LED1 to LED4 to be outputs as
    well as the UART Tx line. */
    AT91C_BASE_PIOB->PIO_PER = LED_MASK; // Set in PIO mode
    AT91C_BASE_PIOB->PIO_OER = LED_MASK; // Configure in Output

    /* Enable the peripheral clock. */
    AT91C_BASE_PMC->PMC_PCER =
        ((1 << AT91C_ID_PIOA) |
         (1 << AT91C_ID_PIOB) |
         (1 << AT91C_ID_SPI0) |
         (1 << AT91C_ID_US0) |
         (1 << AT91C_ID_TWI) |
         (1 << AT91C_ID_PWMC) |
         (1 << AT91C_ID_UDP) |
         (1 << AT91C_ID_EMAC) );

    //setup gpios
    PIO_Configure(pins, NUMBER_OF_SYSTEM_PINS);

    //config PIOA
    AT91C_BASE_PIOA->PIO_PER = 0x7ff883e4;
    AT91C_BASE_PIOA->PIO_PDR = 0x00077c1b;

    AT91C_BASE_PIOA->PIO_OER = 0x4d800204;
    AT91C_BASE_PIOA->PIO_ODR = 0x327ffdfb;
    
    AT91C_BASE_PIOA->PIO_IFER = 0x00780000;
    AT91C_BASE_PIOA->PIO_IFDR = 0x7f87ffff;

    AT91C_BASE_PIOA->PIO_SODR = 0x00000000;
    AT91C_BASE_PIOA->PIO_CODR = 0x7fffffff;

    AT91C_BASE_PIOA->PIO_IER = 0x00000000;
    AT91C_BASE_PIOA->PIO_IDR = 0x7fffffff;

    AT91C_BASE_PIOA->PIO_MDER = 0x00000c00;
    AT91C_BASE_PIOA->PIO_MDDR = 0x7ffff3ff;

    AT91C_BASE_PIOA->PIO_PPUER = 0x00000000;
    AT91C_BASE_PIOA->PIO_PPUDR = 0x7fffffff;

    AT91C_BASE_PIOA->PIO_ASR = 0x00077c1b;
    AT91C_BASE_PIOA->PIO_BSR = 0x00000000;

    AT91C_BASE_PIOA->PIO_OWER = 0x000001e0;
    AT91C_BASE_PIOA->PIO_OWDR = 0x7ffffe1f;

/* Re-enable user reset*/
    AT91C_BASE_RSTC->RSTC_RMR = (0xA5000000 | AT91C_RSTC_URSTEN);
    //turn on the pwm
    pwm_init();

    //init the i2c
    I2CInitialize();

    //turn on the ADC, no trigger (software trig only), 1MHz CLK, 20us startup, 500ns setup/hold
    ADC_Initialize(AT91C_BASE_ADC, AT91C_ID_ADC, AT91C_ADC_TRGEN_DIS, 0, AT91C_ADC_SLEEP_NORMAL_MODE, AT91C_ADC_LOWRES_10_BIT, MCK, 1000000, 20, 500);
    //turn on channel 4
    ADC_EnableChannel(AT91C_BASE_ADC, 4);

}

/*------------------------------------------------------------*/
/**
	Idle hook is run when nothing else is running
 */
void vApplicationIdleHook( void ) {
	/*LED_ERROR_OFF;
    LED_STABLE_OFF;
    LED_HEATON_OFF;
    LED_FANON_ON;*/
}


