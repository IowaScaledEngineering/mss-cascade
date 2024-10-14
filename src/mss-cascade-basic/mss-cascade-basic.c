/*************************************************************************
Title:    MSS-CASCADE-BASIC
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
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

#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>

#include "debouncer.h"
#include "optionSwitches.h"
#include "signalHead.h"
#include "mss.h"

#define LOOP_UPDATE_TIME_MS       50
#define STARTUP_LOCKOUT_TIME_MS  500

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

volatile uint8_t signalHeadOptions = 0;
volatile uint32_t millis = 0;
SignalState_t signalA;
SignalState_t signalB;

// A few hardware definitions

// MSS Port Connections
// These are in the order of:
//  Adjacent address, bitmask
//  Approach address, bitmask
//  Adv Approach address, bitmask
//  Divering Approach address, bitmask

#define MSS_PORT_A_DEF   &PINA, _BV(4), &PINA, _BV(3), &PINA, _BV(2), NULL, _BV(0)
#define MSS_PORT_B_DEF   &PINA, _BV(7), &PINA, _BV(6), &PINA, _BV(5), NULL, _BV(0)

// Signal Port Connections
// These are in the order of:
//  Red address, bitmask
//  Yellow address, bitmask
//  Green address, bitmask

#define SIGNAL_HEAD_A_DEF   &PORTB, _BV(PB2), &PORTB, _BV(PB1), &PORTB, _BV(PB0)
#define SIGNAL_HEAD_B_DEF   &PORTB, _BV(PB4), &PORTB, _BV(PB5), &PORTB, _BV(PB6)


uint32_t getMillis()
{
	uint32_t retmillis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}
	return retmillis;
}

ISR(TIMER0_COMPA_vect) 
{
	static uint8_t flasherCounter = 0;
	static uint8_t flasher = 0;
	static uint8_t pwmPhase = 0;
	static uint8_t subMillisCounter = 0;
	
	// The ISR does two main things - updates the LED outputs since
	//  PWM is done through software, and updates millis which is used
	//  to trigger various events
	// We need this to run at roughly 125 Hz * number of PWM levels (32).  That makes a nice round 4kHz
	
	// First thing, output the signals so that the PWM doesn't get too much jitter

	signalHeadISR_OutputPWM(&signalA, signalHeadOptions, pwmPhase, SIGNAL_HEAD_A_DEF);
	signalHeadISR_OutputPWM(&signalB, signalHeadOptions, pwmPhase, SIGNAL_HEAD_B_DEF);

	// Now do all the counter incrementing and such
	if (++subMillisCounter >= 4)
	{
		subMillisCounter = 0;
		millis++;
	}

	pwmPhase = (pwmPhase + 1) & 0x1F;

	if (0 == pwmPhase)
	{
		pwmPhase = 0;
		flasherCounter++;
		if (flasherCounter > 94)
		{
			flasher ^= 0x01;
			flasherCounter = 0;
		}

		// We rolled over the PWM counter, calculate the next PWM widths
		// This runs at 125 frames/second essentially

		signalHeadISR_AspectToNextPWM(&signalA, flasher, signalHeadOptions);
		signalHeadISR_AspectToNextPWM(&signalB, flasher, signalHeadOptions);
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

int main(void)
{
	MSSPort_t mssPortA;
	MSSPort_t mssPortB;
	DebounceState8_t optionsDebouncer;
	uint32_t lastReadTime = 0;
	uint32_t currentTime = 0;
	uint8_t initialLockout = (STARTUP_LOCKOUT_TIME_MS)/(LOOP_UPDATE_TIME_MS);
	uint8_t optionJumpers = 0;
	
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
	//  PA1 - Analog - Options 1
	//  PA0 - Analog - Options 0

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Output - B Signal - GREEN  (low=active)
	//  PB5 - Output - B Signal - YELLOW (low=active)
	//  PB5 - Output - B Signal - RED (low=active)
	//  PB3 - Input  - Common anode / common cathode sense (1 = common anode)
	//  PB2 - Output - A Signal - GREEN (low=active)
	//  PB1 - Output - A Signal - YELLOW (low=active)
	//  PB0 - Output - A Signal - RED (low=active)

	PORTA = 0b11111100;
	DDRA  = 0b00000000;

	PORTB = 0b11111111;
	DDRB  = 0b01110111;

	initializeTimer();
	initializeOptions(&optionsDebouncer);

	signalHeadInitialize(&signalA);
	signalHeadInitialize(&signalB);

	signalHeadAspectSet(&signalA, ASPECT_OFF);
	signalHeadAspectSet(&signalB, ASPECT_OFF);
	
	mssPortInitialize(&mssPortA);
	mssPortInitialize(&mssPortB);

	// Need to set common anode or common cathode
	// as quickly as possible to prevent an initialization flash
	// initializeOptions() has already gotten the initial state of the CA/CC line
	optionJumpers = getDebouncedState(&optionsDebouncer);
	if (optionJumpers & OPTION_COMMON_ANODE)
		signalHeadOptions |= SIGNAL_OPTION_COMMON_ANODE;
	else 
		signalHeadOptions &= ~SIGNAL_OPTION_COMMON_ANODE;

	sei();
	wdt_reset();


	while(1)
	{
		wdt_reset();

		currentTime = getMillis();

		// Because debouncing and such is built into option reading and the MSS library, only 
		//  run the updates every 10mS or so.

		if (((uint32_t)currentTime - lastReadTime) > LOOP_UPDATE_TIME_MS)
		{
			uint8_t mssOptions = 0;
			
			SignalAspect_t aspect;
			lastReadTime = currentTime;

			readOptions(&optionsDebouncer);
			optionJumpers = getDebouncedState(&optionsDebouncer);
//			optionJumpers |= OPTION_B_FOUR_ASPECT;// | OPTION_COMMON_ANODE | OPTION_C_SEARCHLIGHT_MODE;

			// Convert global option bits to signal head option bits
			if (optionJumpers & OPTION_C_SEARCHLIGHT_MODE)
				signalHeadOptions |= SIGNAL_OPTION_SEARCHLIGHT;
			else 
				signalHeadOptions &= ~SIGNAL_OPTION_SEARCHLIGHT;

			if (optionJumpers & OPTION_COMMON_ANODE)
				signalHeadOptions |= SIGNAL_OPTION_COMMON_ANODE;
			else 
				signalHeadOptions &= ~SIGNAL_OPTION_COMMON_ANODE;

			// Convert global option bits to MSS option bits
			if (optionJumpers & OPTION_A_APPROACH_LIGHTING)
				mssOptions |= MSS_ASPECT_OPTION_APPRCH_LIGHTING;

			if (optionJumpers & OPTION_B_FOUR_ASPECT)
				mssOptions |= MSS_ASPECT_OPTION_FOUR_INDICATION;


			// Read state of MSS bus ports coming in
			mssReadPort(&mssPortA, MSS_PORT_A_DEF);
			mssReadPort(&mssPortB, MSS_PORT_B_DEF);


			// The purpose of the initialLockout is to give the MSS lines and debouncers time to stabilize before we bring the signals up
			// It just looks cleaner to the user than the signals bouncing all over the place as the lines settle.
			if (0 != initialLockout)
			{
				initialLockout--;
				continue;
			}

			mssIndicationToSingleHeadAspect(mssPortA.indication, &aspect, mssOptions, mssPortApproach(&mssPortB));
			signalHeadAspectSet(&signalB, aspect);
			
			mssIndicationToSingleHeadAspect(mssPortB.indication, &aspect, mssOptions, mssPortApproach(&mssPortA));
			signalHeadAspectSet(&signalA, aspect);
		}

	}
}




