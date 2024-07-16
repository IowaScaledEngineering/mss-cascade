/*************************************************************************
Title:    MSS-CASCADE-BASIC
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
          Based on the work of David Johnson-Davies - www.technoblogy.com - 23rd October 2017
           and used under his Creative Commons Attribution 4.0 International license
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>
#include "spiflash.h"
#include "audioAndLights.h"
#include "ispl.h"
#include "debouncer.h"

#define MAX_TRACKS 6

DebounceState8_t inputDebouncer;
DebounceState8_t mssDebouncer;

void readInputs()
{
	static uint32_t lastRead = 0;
	uint8_t currentInputState = 0;
	uint32_t currentMillis = getMillis();
	if (((uint32_t)currentMillis - lastRead) > 10)
	{
		lastRead = currentMillis;

		// Inputs (bit / io / name):  
		//  0 - PA4 - Enable 1
		//  1 - PB0 - SW1
		//  2 - PB2 - SW2
		//  3 - PB4 - SW3
		//  4 - PB5 - SW4
		//  5 - PB6 - SW5
		currentInputState = ~(((PINA & _BV(PA4))>>4) | ((PINB & _BV(PB0))<<1) | (PINB & _BV(PB2)) | ((PINB & (_BV(PB4) | _BV(PB5) | _BV(PB6)))>>1));
		debounce8(currentInputState, &inputDebouncer);
	} 
}


typedef enum 
{
	INDICATION_STOP               = 0,
	INDICATION_APPROACH           = 1,
	INDICATION_APPROACH_DIVERGING = 2,
	INDICATION_ADVANCE_APPROACH   = 3,
	INDICATION_CLEAR              = 4
} MSSPortIndication_t;

typedef struct
{
	MSSPortIndication_t indication;
	DebounceState8_t debounce;
} MSSPort_t;

typedef struct
{
	uint8_t* adjacentPort;
	uint8_t adjacentPin;
	uint8_t* approachPort;
	uint8_t approachPin;
	uint8_t* advApproachPort;
	uint8_t advApproachPin;
	uint8_t* divApproachPort;
	uint8_t divApproachPin;
} MSSPortPins_t;



	//  PA7 - Input  - B MSS - Adjacent
	//  PA6 - Input  - B MSS - Approach
	//  PA5 - Input  - B MSS - Advance Approach
	//  PA4 - Input  - A MSS - Adjacent
	//  PA3 - Input  - A MSS - Approach
	//  PA2 - Input  - A MSS - Advance Approach

#define MSS_MASK_ADJACENT      0x01
#define MSS_MASK_APPROACH      0x02
#define MSS_MASK_ADV_APPROACH  0x04
#define MSS_MASK_DIV_APPROACH  0x08

void mssReadPort(MSSPort_t* port, const MSSPortPins_t* const pins)
{
	uint8_t mssInputState = 0;
	
	if ( *(port->adjacentPort) & (1<<port->adjacentPin) )
		mssInputState |= MSS_MASK_ADJACENT;

	if ( *(port->approachPin) & (1<<port->approachPin) )
		mssInputState |= MSS_MASK_APPROACH;

	if ( *(port->advApproachPort) & (1<<port->advApproachPin) )
		mssInputState |= MSS_MASK_ADV_APPROACH;
		
	if ( NULL != port->divApproachPort && (*(port->divApproachPort) & (1<<port->divApproachPin)) )
		mssInputState |= MSS_MASK_DIV_APPROACH;
		

	debounce8(mssInputState, &(port->debounce));

	mssInputState = getDebouncedState(&(port->debounce));
	
	if (mssInputState & MSS_MASK_ADJACENT)
	{
		port->indication = INDICATION_STOP;
	}
	else if (mssInputState & MSS_MASK_APPROACH)
	{
		if (mssInputState & MSS_MASK_DIV_APPROACH)
			port->indication = INDICATION_APPROACH_DIVERGING;
		else
			port->indication = INDICATION_APPROACH;
	}
	else if (mssInputState & MSS_MASK_ADV_APPROACH)
	{
		port->indication = INDICATION_ADVANCE_APPROACH;
	}
	else
	{
		port->indication = INDICATION_CLEAR;
	}
}



void mss_readCodeline()
{
	static uint32_t lastRead = 0;
	uint8_t currentInputState = 0;
	uint32_t currentMillis = getMillis();
	if (((uint32_t)currentMillis - lastRead) > 10)
	{
		lastRead = currentMillis;

		// Inputs (bit / io / name):  
		//  0 - PA4 - Enable 1
		//  1 - PB0 - SW1
		//  2 - PB2 - SW2
		//  3 - PB4 - SW3
		//  4 - PB5 - SW4
		//  5 - PB6 - SW5
		currentInputState = ~(((PINA & _BV(PA4))>>4) | ((PINB & _BV(PB0))<<1) | (PINB & _BV(PB2)) | ((PINB & (_BV(PB4) | _BV(PB5) | _BV(PB6)))>>1));
		debounce8(currentInputState, &inputDebouncer);
	} 
}



int main(void)
{
	MSSPort_t mssPortA;
	MSSPort_t mssPortB;
	const MSSPortPins_t const mssPortAPins = { &PORTA, 4, &PORTA, 3, &PORTA, 2, NULL, 0 };
	const MSSPortPins_t const mssPortBPins = { &PORTA, 7, &PORTA, 6, &PORTA, 5, NULL, 0 };

	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	// PORT A
	//  PA7 - Input  - B MSS - Adjacent
	//  PA6 - Input  - B MSS - Approach
	//  PA5 - Input  - B MSS - Advance Approach
	//  PA4 - Input  - A MSS - Adjacent
	//  PA3 - Input  - A MSS - Approach
	//  PA2 - Input  - A MSS - Advance Approach
	//  PA1 - Output - A Signal - YELLOW (low=active)
	//  PA0 - Output - A Signal - RED (low=active)

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Output - B Signal - GREEN  (low=active)
	//  PB5 - Output - B Signal - YELLOW (low=active)
	//  PB5 - Output - B Signal - RED (low=active)
	//  PB3 - Output - A Signal - GREEN (low=active)
	//  PB2 - Input  - Configuration Jumper A
	//  PB1 - Input  - Configuration Jumper B
	//  PB0 - Input  - Configuration Jumper C

	PORTA = 0b11111111;
	DDRA  = 0b00000011;

	PORTB = 0b11111111;
	DDRB  = 0b01111000;

	initDebounceState8(&inputDebouncer, 0x00);
	initDebounceState8(&mssDebouncer, 0x00);

	sei();
	wdt_reset();

	while(1)
	{
		wdt_reset();

		// if time to read...
		mssReadPort(&mssPortA, &mssPortAPins);
		mssReadPort(&mssPortB, &mssPortBPins);
	}
}




