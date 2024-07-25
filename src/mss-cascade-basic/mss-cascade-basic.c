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

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

uint8_t globalOptions = 0;

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
#define MSS_ASPECT_OPTION_SEARCHLIGHT       0x04

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

typedef struct
{
	MSSSignalAspect_t startAspect;
	MSSSignalAspect_t endAspect;
	uint8_t phase;
	uint8_t redPWM;
	uint8_t yellowPWM;
	uint8_t greenPWM;
} SignalState_t;

bool isGreenToYellow(MSSSignalAspect_t startAspect, MSSSignalAspect_t endAspect)
{
	if ((startAspect == ASPECT_GREEN || startAspect == ASPECT_FL_GREEN) 
		&& (endAspect == ASPECT_YELLOW || endAspect == ASPECT_FL_YELLOW))
		return true;
	return false;
}

bool isYellowToGreen(MSSSignalAspect_t startAspect, MSSSignalAspect_t endAspect)
{
	if ((startAspect == ASPECT_YELLOW || startAspect == ASPECT_FL_YELLOW)
			&& (endAspect == ASPECT_GREEN || endAspect == ASPECT_FL_GREEN))
		return true;
	return false;
}

const uint16_t const searchlightPWMsThroughRed[32] PROGMEM = { 
	27648,
	17408,
	12288,
	0,
	17,
	25,
	25,
	17,
	0,
	384,
	544,
	551,
	544,
	544,
	704,
	704,
	544,
	544,
	544,
	551,
	544,
	384,
	236,
	17,
	22,
	17,
	236,
	384,
	544,
	704,
	864,
	992}; 


const uint16_t const searchlightPWMsInvolvingRed[] PROGMEM = { 
	31744,
	27648,
	22528,
	17408,
	12288,
	5120,
	0,
	160,
	384,
	544,
	704,
	864,
	992 }; 

void isr_AspectToOutputs(MSSSignalAspect_t signalAspect, SignalState_t* sig, uint8_t flasher, uint8_t options)
{
	bool searchlightMode = false;
	
	if (MSS_ASPECT_OPTION_SEARCHLIGHT & options)
		searchlightMode = true;
	
	// If it's a flashing aspect, mux the flasher in with the color
	if (signalAspect == ASPECT_FL_GREEN || signalAspect == ASPECT_FL_YELLOW || signalAspect == ASPECT_FL_RED)
		signalAspect = (flasher)?signalAspect:ASPECT_OFF;

	// If we're not currently running a transition and the aspect changed, start the transitioning
	if (sig->startAspect == sig->endAspect)
	{
		sig->phase = 0;
		sig->endAspect = signalAspect;
	}

	if (sig->startAspect != sig->endAspect)
	{
		// We're in transition towards the end aspect
		// How we do this depends upon the type of signal and the transition being made
		// For searchlights (US&S H, H2, H5 and GRS SA), green to yellow or vice versa passes through red and there's a little bounce giving
		//  a second red, but there's no long fade because the bulb never turns off.  
		// For searchlights passing between any color and red (or vice versa), it's just a quick bounce as the roundels move - no fade
		// For all other signals and searchlights going on or off, there's a fade in/out

		if (searchlightMode && sig->startAspect != ASPECT_OFF && sig->endAspect != ASPECT_OFF)
		{
			uint8_t idx = sig->phase;

			if (isGreenToYellow(sig->startAspect, sig->endAspect) || isYellowToGreen(sig->startAspect, sig->endAspect))
			{
				// *****************
				// Searchlight changing yellow-green or green-yellow through red
				// *****************

				// These are the special ones that bounce through red (at 60 frames/sec measured off my H2)
				// Yellow to green
				// ~6 frames to red
				// ~6 frames to green
				// ~12 frames back to red
				// ~8 frames to green
				
				// This uint16 is comprised of:
				//  0:4 - red channel
				//  5:9 - up channel
				//  10:14 - down channel
				
				uint16_t pwmWord = pgm_read_word(&searchlightPWMsThroughRed[idx]);

				sig->redPWM = pwmWord & 0x1F;
				if (isGreenToYellow(sig->startAspect, sig->endAspect))
				{
					sig->greenPWM = (pwmWord>>10) & 0x1F;
					sig->yellowPWM = (pwmWord>>5) & 0x1F;
				} else {
					sig->greenPWM = (pwmWord>>5) & 0x1F;
					sig->yellowPWM = (pwmWord>>10) & 0x1F;
				}
				sig->phase++;
				if (sig->phase == sizeof(searchlightPWMsThroughRed)/sizeof(uint16_t))
				{
					// We're done
					sig->startAspect = sig->endAspect;
				}
			}
			else
			{
				// *****************
				// Searchlight changing from yellow or green to red, or red to yellow or green
				// *****************
				uint16_t pwmWord = pgm_read_word(&searchlightPWMsInvolvingRed[idx]);
				
				uint8_t upPhase = pwmWord>>5;
				uint8_t downPhase = pwmWord>>10;

				sig->redPWM = sig->yellowPWM = sig->greenPWM = 0;

				switch(sig->startAspect)
				{
					case ASPECT_RED:
					case ASPECT_FL_RED:
						sig->redPWM = downPhase;
						break;

					case ASPECT_YELLOW:
					case ASPECT_FL_YELLOW:
						sig->yellowPWM = downPhase;
						break;

					case ASPECT_GREEN:
					case ASPECT_FL_GREEN:
						sig->greenPWM = downPhase;
						break;

					default:
						break;
				}

				switch(sig->endAspect)
				{
					case ASPECT_RED:
					case ASPECT_FL_RED:
						sig->redPWM = upPhase;
						break;

					case ASPECT_YELLOW:
					case ASPECT_FL_YELLOW:
						sig->yellowPWM = upPhase;
						break;

					case ASPECT_GREEN:
					case ASPECT_FL_GREEN:
						sig->greenPWM = upPhase;
						break;

					default:
						break;
				}

				sig->phase++;
				if (sig->phase == sizeof(searchlightPWMsInvolvingRed)/sizeof(uint16_t))
				{
					// We're done
					sig->phase = 0;
					sig->startAspect = sig->endAspect;
				}
			}

		} else {
			// *****************
			// All other signals, where the bulbs fade in/out (and searchlights to or from off)
			// *****************
			uint8_t targetPWMRed = 0, targetPWMYellow = 0, targetPWMGreen = 0;
			uint8_t finalState = 0;
			
			switch(sig->endAspect)
			{
				case ASPECT_RED:
				case ASPECT_FL_RED:
					targetPWMRed = 0x1F;
					break;

				case ASPECT_YELLOW:
				case ASPECT_FL_YELLOW:
					targetPWMYellow = 0x1F;
					break;
					
				case ASPECT_GREEN:
				case ASPECT_FL_GREEN:
					targetPWMGreen = 0x1F;
					break;

				default:
					break;
			}
			
			if (targetPWMRed > sig->redPWM)
				sig->redPWM = MIN(sig->redPWM + 2, 0x1F);
			else if (targetPWMRed < sig->redPWM )
				sig->redPWM -= MIN(2, sig->redPWM);
			else
				finalState += 1;

			if (targetPWMYellow > sig->yellowPWM)
				sig->yellowPWM = MIN(sig->yellowPWM + 2, 0x1F);
			else if (targetPWMYellow < sig->yellowPWM)
				sig->yellowPWM -= MIN(2, sig->yellowPWM);
			else
				finalState += 1;

			if (targetPWMGreen > sig->greenPWM)
				sig->greenPWM = MIN(sig->greenPWM + 2, 0x1F);
			else if (targetPWMGreen < sig->greenPWM)
				sig->greenPWM -= MIN(2, sig->greenPWM);
			else
				finalState += 1;

			if (3 == finalState)
			{
				sig->phase = 0;
				sig->startAspect = sig->endAspect;
			}
		}

	} else {
		// We're at steady state and the signal isn't changing, so 
		// just set the PWM based on the aspect for safety
		sig->redPWM = sig->yellowPWM = sig->greenPWM = 0;
		switch(sig->startAspect)
		{
			case ASPECT_RED:
			case ASPECT_FL_RED:
				sig->redPWM = 0x1F;
				break;

			case ASPECT_YELLOW:
			case ASPECT_FL_YELLOW:
				sig->yellowPWM = 0x1F;
				break;

			case ASPECT_GREEN:
			case ASPECT_FL_GREEN:
				sig->greenPWM = 0x1F;
				break;

			default:
				break;
		}
	}
}

ISR(TIMER0_COMPA_vect) 
{
	static uint8_t flasherCounter = 0;
	static uint8_t flasher = 0;
	static uint8_t pwmCounter = 0;
	static uint8_t subMillisCounter = 0;
	static SignalState_t sigA = {ASPECT_OFF, ASPECT_OFF, 0, 0, 0, 0};
	static SignalState_t sigB = {ASPECT_OFF, ASPECT_OFF, 0, 0, 0, 0};


	// The ISR does two main things - updates the LED outputs since
	//  PWM is done through software, and updates millis which is used
	//  to trigger various events
	// We need this to run at roughly 125 Hz * number of PWM levels.  If we assume 32, that makes a nice round 4kHz

	// First, set all the PWMs so as to minimize jitter
	if (sigA.redPWM > pwmCounter)
		PORTA &= ~_BV(PA0);
	else
		PORTA |= _BV(PA0);

	if (sigA.yellowPWM > pwmCounter)
		PORTA &= ~_BV(PA1);
	else
		PORTA |= _BV(PA1);

	if (sigA.greenPWM > pwmCounter)
		PORTB &= ~_BV(PB3);
	else
		PORTB |= _BV(PB3);

	if (sigB.redPWM > pwmCounter)
		PORTB &= ~_BV(PB4);
	else
		PORTB |= _BV(PB4);

	if (sigB.yellowPWM > pwmCounter)
		PORTB &= ~_BV(PB5);
	else
		PORTB |= _BV(PB5);

	if (sigB.greenPWM > pwmCounter)
		PORTB &= ~_BV(PB6);
	else
		PORTB |= _BV(PB6);

	// Now do all the counter incrementing and such
	if (++subMillisCounter >= 4)
	{
		subMillisCounter = 0;
		millis++;
	}

	if (++pwmCounter >= 32)
	{
		pwmCounter = 0;
		flasherCounter++;
		if (flasherCounter > 94)
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
		isr_AspectToOutputs(aspectSignalA, &sigA, flasher, globalOptions);
		isr_AspectToOutputs(aspectSignalB, &sigB, flasher, globalOptions);
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
#define OPTION_C_SEARCHLIGHT_MODE    0x01

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

			if (getDebouncedState(&optionsDebouncer) & OPTION_C_SEARCHLIGHT_MODE)
				options |= MSS_ASPECT_OPTION_SEARCHLIGHT;

//			options = MSS_ASPECT_OPTION_FOUR_INDICATION | MSS_ASPECT_OPTION_SEARCHLIGHT;// | MSS_ASPECT_OPTION_APPRCH_LIGHTING;

			globalOptions = options;  // Make this atomic;

			mssIndicationToSingleHeadAspect(mssPortA.indication, &aspectSignalB, globalOptions, mssPortApproach(&mssPortB));
			mssIndicationToSingleHeadAspect(mssPortB.indication, &aspectSignalA, globalOptions, mssPortApproach(&mssPortA));
		}

	}
}




