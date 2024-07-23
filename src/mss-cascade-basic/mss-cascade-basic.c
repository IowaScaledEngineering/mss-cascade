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
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>
#include "debouncer.h"

//  Definitions:
//  Physical   - The state of the MSS wires
//  Indication - The "meaning" to be conveyed to a train viewing the signal
//  Aspect     - The appearance of the signal - color, flashing, etc.

volatile uint32_t millis = 0;
uint32_t getMillis()
{
	uint32_t retmillis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}

	return retmillis;
}


typedef enum
{
	ASPECT_OFF          = 0,
	ASPECT_GREEN        = 1,
	ASPECT_YELLOW       = 2,
	ASPECT_FL_YELLOW    = 3,
	ASPECT_RED          = 4,
	ASPECT_FL_GREEN     = 5,
	ASPECT_FL_RED       = 6,
	ASPECT_LUNAR        = 7
} MSSSignalAspect_t;

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
	volatile uint8_t* adjacentPort;
	uint8_t adjacentPin;
	volatile uint8_t* approachPort;
	uint8_t approachPin;
	volatile uint8_t* advApproachPort;
	uint8_t advApproachPin;
	volatile uint8_t* divApproachPort;
	uint8_t divApproachPin;
} MSSPortPins_t;

MSSSignalAspect_t aspectSignalA;
MSSSignalAspect_t aspectSignalB;

#define MSS_MASK_ADJACENT      0x01
#define MSS_MASK_APPROACH      0x02
#define MSS_MASK_ADV_APPROACH  0x04
#define MSS_MASK_DIV_APPROACH  0x08

void mssPortInitialize(MSSPort_t* port)
{
	initDebounceState8(&port->debounce, 0x00);
	port->indication = INDICATION_STOP;
}

bool mssPortApproach(MSSPort_t* port)
{
	switch(port->indication)
	{
		case INDICATION_STOP:
		case INDICATION_APPROACH:
			return true;

		default:
			break;
	}
	return false;
}

void mssReadPort(MSSPort_t* port, const MSSPortPins_t* const pins)
{
	uint8_t mssInputState = 0;
	
	if ( *(pins->adjacentPort) & (1<<pins->adjacentPin) )
		mssInputState |= MSS_MASK_ADJACENT;

	if ( *(pins->approachPort) & (1<<pins->approachPin) )
		mssInputState |= MSS_MASK_APPROACH;

	if ( *(pins->advApproachPort) & (1<<pins->advApproachPin) )
		mssInputState |= MSS_MASK_ADV_APPROACH;
		
	if ( NULL != pins->divApproachPort && (*(pins->divApproachPort) & (1<<pins->divApproachPin)) )
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

#define MSS_ASPECT_OPTION_APPRCH_LIGHTING   0x01
#define MSS_ASPECT_OPTION_FOUR_INDICATION   0x02
void mssIndicationToSingleHeadAspect(MSSPortIndication_t indication, MSSSignalAspect_t* aspect, uint8_t options, bool approachActive)
{
	// If we're approach lit and there's nothing on approach, turn off
	if (!approachActive && (options & MSS_ASPECT_OPTION_APPRCH_LIGHTING))
	{
		*aspect = ASPECT_OFF;
		return;
	}

	switch (indication)
	{
		case INDICATION_STOP:
		default:
			*aspect = ASPECT_RED;
			break;

		case INDICATION_APPROACH:
		case INDICATION_APPROACH_DIVERGING:
			*aspect = ASPECT_YELLOW;
			break;

		case INDICATION_ADVANCE_APPROACH:
			*aspect = (options & MSS_ASPECT_OPTION_FOUR_INDICATION)?ASPECT_FL_YELLOW:ASPECT_GREEN;
			break;

		case INDICATION_CLEAR:
			*aspect = ASPECT_GREEN;
			break;
	}
}


void isr_AspectToOutputs(MSSSignalAspect_t signalAspect, uint8_t *redPWM, uint8_t* yellowPWM, uint8_t* greenPWM, uint8_t flasher)
{
	uint8_t targetPWMRed = 0;
	uint8_t targetPWMYellow = 0;
	uint8_t targetPWMGreen = 0;
	
	switch(signalAspect)
	{
		case ASPECT_RED:
			targetPWMRed = 0x1F;
			break;

		case ASPECT_FL_RED:
			targetPWMRed = (flasher)?0x1F:0x00;
			break;

		case ASPECT_YELLOW:
			targetPWMYellow = 0x1F;
			break;

		case ASPECT_FL_YELLOW:
			targetPWMYellow = (flasher)?0x1F:0x00;
			break;
			
		case ASPECT_GREEN:
			targetPWMGreen = 0x1F;
			break;

		case ASPECT_FL_GREEN:
			targetPWMGreen = (flasher)?0x1F:0x00;
			break;

		default:
			break;
	}

	if (targetPWMRed > *redPWM)
		(*redPWM)++;
	else if (*redPWM != 0)
		(*redPWM)--;

	if (targetPWMYellow > *yellowPWM)
		(*yellowPWM)++;
	else if (*yellowPWM != 0)
		(*yellowPWM)--;

	if (targetPWMGreen > *greenPWM)
		(*greenPWM)++;
	else if (*greenPWM != 0)
		(*greenPWM)--;
}


ISR(TIMER0_COMPA_vect) 
{
	static uint8_t sigANextRed = 0;
	static uint8_t sigANextYellow = 0;
	static uint8_t sigANextGreen = 0;

	static uint8_t sigBNextRed = 0;
	static uint8_t sigBNextYellow = 0;
	static uint8_t sigBNextGreen = 0;
	static uint8_t flasherCounter = 0;
	static uint8_t flasher = 0;
	static uint8_t pwmCounter = 0;
	static uint8_t subMillisCounter = 0;
	
	// The ISR does two main things - updates the LED outputs since
	//  PWM is done through software, and updates millis which is used
	//  to trigger various events
	// We need this to run at roughly 125 Hz * number of PWM levels.  If we assume 32, that makes a nice round 4kHz

	// First, set all the PWMs so as to minimize jitter
	if (sigANextRed > pwmCounter)
		PORTA &= ~_BV(PA0);
	else
		PORTA |= _BV(PA0);

	if (sigANextYellow > pwmCounter)
		PORTA &= ~_BV(PA1);
	else
		PORTA |= _BV(PA1);

	if (sigANextGreen > pwmCounter)
		PORTB &= ~_BV(PB3);
	else
		PORTB |= _BV(PB3);

	if (sigBNextRed > pwmCounter)
		PORTB &= ~_BV(PB4);
	else
		PORTB |= _BV(PB4);

	if (sigBNextYellow > pwmCounter)
		PORTB &= ~_BV(PB5);
	else
		PORTB |= _BV(PB5);

	if (sigBNextGreen > pwmCounter)
		PORTB &= ~_BV(PB6);
	else
		PORTB |= _BV(PB6);

	// Now do all the counter incrementing and such
	if (++subMillisCounter >= 4)
	{
		subMillisCounter = 0;
		millis++;
	}

	pwmCounter = (pwmCounter + 1) & 0x1F;
	if (++pwmCounter >= 32)
	{
		pwmCounter = 0;
		flasherCounter++;
		if (flasherCounter > 187)
		{
			flasher ^= 0x01;
			flasherCounter = 0;
		}

		// We rolled over the PWM counter, calculate the next PWM widths
		// This runs at 125 frames/second essentially
		// Searchlight logic
		// Dark to on takes ~1/3 second, or 42 frames
		// Changeovers take 33 frames
		// End to end (yellow to green, green to yellow)
		//   12 frames to red
		//   12 frames to target color
		//   25 frames back to red
		//   15 frames back to target color

		isr_AspectToOutputs(aspectSignalA, &sigANextRed, &sigANextYellow, &sigANextGreen, flasher);
		isr_AspectToOutputs(aspectSignalB, &sigBNextRed, &sigBNextYellow, &sigBNextGreen, flasher);

	}
}

void initializeTimer()
{
	TIMSK = 0;                                    // Timer interrupts OFF
	// Set up Timer/Counter0 for 100Hz clock
	TCCR0A = 0b00000001;  // CTC Mode
	TCCR0B = 0b00000010;  // CS01 - 1:8 prescaler
	OCR0A = 250;           // 8MHz / 8 / 125 = 8kHz
	TIMSK = _BV(OCIE0A);
}

#define OPTION_A_APPROACH_LIGHTING   0x04
#define OPTION_B_FOUR_ASPECT         0x02
#define OPTION_C_RESERVED            0x01

void readOptions(DebounceState8_t* optionsDebouncer)
{
	// Inputs (bit / io / name):  

	//  1 - PB0 - Option Jumper C (JP7)
	//  2 - PB2 - Option Jumper B (JP8)
	//  3 - PB4 - Option Jumper A (JP9)

	debounce8(0x07 & (~PINB), optionsDebouncer);
}


int main(void)
{
	MSSPort_t mssPortA;
	MSSPort_t mssPortB;
	DebounceState8_t optionsDebouncer;
	uint32_t lastReadTime = 0;
	uint32_t currentTime = 0;
	const MSSPortPins_t const mssPortAPins = { &PINA, 4, &PINA, 3, &PINA, 2, NULL, 0 };
	const MSSPortPins_t const mssPortBPins = { &PINA, 7, &PINA, 6, &PINA, 5, NULL, 0 };

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

	initializeTimer();
	initDebounceState8(&optionsDebouncer, 0x00);


	mssPortInitialize(&mssPortA);
	mssPortInitialize(&mssPortB);
	sei();
	wdt_reset();

	aspectSignalA = ASPECT_RED;
	aspectSignalB = ASPECT_RED;

	while(1)
	{
		wdt_reset();

		currentTime = getMillis();
		if (((uint32_t)currentTime - lastReadTime) > 10)
		{
			uint8_t options = 0;

			// if time to read...
			lastReadTime = currentTime;

			readOptions(&optionsDebouncer);
			mssReadPort(&mssPortA, &mssPortAPins);
			mssReadPort(&mssPortB, &mssPortBPins);

			if (getDebouncedState(&optionsDebouncer) & OPTION_A_APPROACH_LIGHTING)
				options |= MSS_ASPECT_OPTION_APPRCH_LIGHTING;

			if (getDebouncedState(&optionsDebouncer) & OPTION_B_FOUR_ASPECT)
				options |= MSS_ASPECT_OPTION_FOUR_INDICATION;

//			options = MSS_ASPECT_OPTION_FOUR_INDICATION | MSS_ASPECT_OPTION_APPRCH_LIGHTING;

			mssIndicationToSingleHeadAspect(mssPortA.indication, &aspectSignalB, options, mssPortApproach(&mssPortB));
			mssIndicationToSingleHeadAspect(mssPortB.indication, &aspectSignalA, options, mssPortApproach(&mssPortA));
		}

	}
}




