///Board.h
/** \file
	Contains basic definitions that are specific to this board, things
	such as LED and button macros are in this file.
 */
	
#ifndef Board_h
#define Board_h

#include "AT91SAM7X256.h"
#include "ioat91sam7x256.h"

#define true	-1
#define false	0

/*-------------------------------*/
/* SAM7Board Memories Definition */
/*-------------------------------*/
// The AT91SAM7X256 embeds a 64-Kbyte SRAM bank, and 256K-Byte Flash

///Number of flash pages on device
#define  FLASH_PAGE_NB		512
///Size of flash pages on device
#define  FLASH_PAGE_SIZE	128

/*-----------------*/
/* Leds Definition */
/*-----------------*/
///Location of LED 1 (ERROR)
#define LED1            (1<<22)	// PB19
///Location of LED 2 (STABLE)
#define LED2            (1<<23)	// PB20
///Location of LED 3 (HEATON)
#define LED3            (1<<24)	// PB21
///Location of LED 4 (FANON)
#define LED4            (1<<25)	// PB22
#define NB_LED			4

///Turn on the ERROR LED
#define LED_ERROR_ON	(AT91C_BASE_PIOB->PIO_CODR	= LED1)
///Turn off the ERROR LED
#define LED_ERROR_OFF	(AT91C_BASE_PIOB->PIO_SODR	= LED1)

///Turn on the STABLE LED
#define LED_STABLE_ON	(AT91C_BASE_PIOB->PIO_CODR	= LED2)
///Turn off the STABLE LED
#define LED_STABLE_OFF	(AT91C_BASE_PIOB->PIO_SODR	= LED2)

///Turn on the HEATON LED
#define LED_HEATON_ON	(AT91C_BASE_PIOB->PIO_CODR	= LED3)
///Turn off the HEATON LED
#define LED_HEATON_OFF	(AT91C_BASE_PIOB->PIO_SODR	= LED3)

///Turn on the FANON LED
#define LED_FANON_ON	(AT91C_BASE_PIOB->PIO_CODR	= LED4)
///Turn off the FANON LED
#define LED_FANON_OFF	(AT91C_BASE_PIOB->PIO_SODR	= LED4)

///mask of all the LEDS
#define LED_MASK        (LED1|LED2|LED3|LED4)

/*-------------------------*/
/* Push Buttons Definition */
/*-------------------------*/

///Check if up button is pressed (1 == pressed)
#define BUTTON_U	(!(((AT91C_BASE_PIOA->PIO_PDSR) >> 19) & 1))
///Check if down button is pressed (1 == pressed)
#define BUTTON_D	(!(((AT91C_BASE_PIOA->PIO_PDSR) >> 20) & 1))
///Check if left button is pressed (1 == pressed)
#define BUTTON_L	(!(((AT91C_BASE_PIOA->PIO_PDSR) >> 21) & 1))
///Check if right button is pressed (1 == pressed)
#define BUTTON_R	(!(((AT91C_BASE_PIOA->PIO_PDSR) >> 22) & 1))

/*--------------*/
/* Master Clock */
/*--------------*/
/// Exetrnal ocilator MAINCK
#define EXT_OC          18432000   
/// MCK (PLLRC div by 2)
#define MCK             47923200   
/// MCK in kHz
#define MCKKHz          (MCK/1000) //

///Hook to the printf function that redirects to serial
int em_printf(const char *fmt, ...);
///Hook the readline function that redirects to serial
int em_readline(char *line);

///Macro to handle printf
#define printf(args...) em_printf(args)
///Macro to handle readlien
#define readline(x) em_readline(x)

///The default amount of time that the safety timer is set to (in ms)
#define SAFETY_TIMER_DEFAULT 30000
///The max value the safety timer can be set to (in ms)
#define SAFETY_TIMER_MAX	 180000
///The max degree rise that can be set for the safety timer (in 100 * degC)
#define SAFETY_DEGREE_MAX	 2000

//will be 8 chars (no null termination) populated before threads are started
extern unsigned char boardId[];

#endif /* Board_h */
