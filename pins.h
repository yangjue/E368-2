#ifndef PINS_GUARD_H
#define PINS_GUARD_H
/// \file pins.h setup for PIOB pins (PIOA is done with massive register writes)

///Number of pins used in the PIO list (not all the pins because others are setup in other fasions)
#define NUMBER_OF_SYSTEM_PINS PIO_LISTSIZE(pins)

#include <pio.h>

///Pins setup
const Pin pins[] = {
    //Start of PIOA
    {(1 << 0),	 	AT91C_BASE_PIOA,	0,	PIO_PERIPH_A,	PIO_DEFAULT}, //UART
    {(1 << 1),	 	AT91C_BASE_PIOA,	0,	PIO_PERIPH_A,	PIO_DEFAULT}, //UART

    {(1 << 5),	 	AT91C_BASE_PIOA,	1,	PIO_INPUT,		PIO_DEFAULT}, //LCD DAT4 (PA5 PIO)
    {(1 << 6),	 	AT91C_BASE_PIOA,	1,	PIO_INPUT,		PIO_DEFAULT}, //LCD DAT5 (PA6 PIO)
    {(1 << 7),	 	AT91C_BASE_PIOA,	1,	PIO_INPUT,		PIO_DEFAULT}, //LCD DAT6 (PA7 PIO)
    {(1 << 8),	 	AT91C_BASE_PIOA,	1,	PIO_INPUT,		PIO_DEFAULT}, //LCD DAT7 (PA8 PIO)

    //Start of PIOB
    {(1 << 21), 	AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT}, //LCD RS (PB21 PIO)
    {(1 << 20),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT}, //LCD RW (PB20 PIO)
    {(1 << 19),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT}, //LCD E (PB19 PIO)
    {(1 << 22),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT}, //Error led
    {(1 << 23),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT}, //stable led
    {(1 << 24),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT}, //Heat on led
    {(1 << 25),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT}, //Fan on led

    //pwm pins
    {(1 << 27),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_PERIPH_B,	PIO_DEFAULT}, //PWM0
    {(1 << 28),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_PERIPH_B,	PIO_DEFAULT}, //PWM1
    {(1 << 29),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_PERIPH_B,	PIO_DEFAULT}, //PWM2
    {(1 << 30),		AT91C_BASE_PIOB,	AT91C_ID_PIOB,	PIO_PERIPH_B,	PIO_DEFAULT}, //PWM3
};

#endif

