/*************************************************************************
Title:    MSS-CASCADE-ADVANCED
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     mss-cascade-adv.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2025 Michael Petersen & Nathan Holmes

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
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>

#include "debouncer.h"
#include "signalHead.h"
#include "mss.h"
#include "i2c.h"
#include "hardware.h"

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

volatile uint8_t signalHeadOptions = 0;
volatile uint32_t millis = 0;
SignalState_t signalAU;
SignalState_t signalAL;
SignalState_t signalBU;
SignalState_t signalBL;

void calculateAspects(uint8_t mssInputs, uint8_t optionJumpers);

uint32_t getMillis()
{
	uint32_t retmillis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}
	return retmillis;
}

static uint8_t flasher = 0;

ISR(TIMER0_COMPA_vect) 
{
	static uint8_t flasherCounter = 0;
	static uint8_t pwmPhase = 0;
	static uint8_t subMillisCounter = 0;
	
	// The ISR does two main things - updates the LED outputs since
	//  PWM is done through software, and updates millis which is used
	//  to trigger various events
	// We need this to run at roughly 125 Hz * number of PWM levels (32).  That makes a nice round 4kHz
	
	// First thing, output the signals so that the PWM doesn't get too much jitter

	signalHeadISR_OutputPWM(&signalAU, signalHeadOptions, pwmPhase, SIGNAL_HEAD_AU_DEF);
	signalHeadISR_OutputPWM(&signalAL, signalHeadOptions, pwmPhase, SIGNAL_HEAD_AL_DEF);
	signalHeadISR_OutputPWM(&signalBU, signalHeadOptions, pwmPhase, SIGNAL_HEAD_BU_DEF);
	signalHeadISR_OutputPWM(&signalBL, signalHeadOptions, pwmPhase, SIGNAL_HEAD_BL_DEF);

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

		signalHeadISR_AspectToNextPWM(&signalAU, flasher, signalHeadOptions);
		signalHeadISR_AspectToNextPWM(&signalAL, flasher, signalHeadOptions);
		signalHeadISR_AspectToNextPWM(&signalBU, flasher, signalHeadOptions);
		signalHeadISR_AspectToNextPWM(&signalBL, flasher, signalHeadOptions);
	}
}

void initializeTimer()
{
	TIMSK = 0;                                    // Timer interrupts OFF
	// Set up Timer/Counter0 for 100Hz clock
	TCCR0A = 0b00000001;  // CTC Mode
	TCCR0B = 0b00000010;  // CS01 - 1:8 prescaler
	OCR0A = 250;           // 8MHz / 8 / 250 = 4kHz
	TIMSK = _BV(OCIE0A);
}

bool getSwitches(uint8_t* retval)
{
	uint8_t i=0, j=0;
	if (!readByte(TCA9555_ADDR_000, TCA9555_GPIN0, &i))
		return false;

	if (!readByte(TCA9555_ADDR_001, TCA9555_GPIN1, &j))
		return false;

	*retval = 0;
	if (i & SWITCH_SW3_IN)
		*retval |= SWITCHMASK_LEFT;

	if (i & SWITCH_SW4_IN)
		*retval |= SWITCHMASK_RIGHT;

	if (j & SWITCH_SW5_IN)
		*retval |= SWITCHMASK_UPPER_HEAD;

	if (j & SWITCH_SW6_IN)
		*retval |= SWITCHMASK_LOWER_HEAD;

	return true;
}

bool getOptions(uint8_t* retval)
{
	uint8_t i=0;
	if (!readByte(TCA9555_ADDR_000, TCA9555_GPIN0, &i))
		return false;
		
		
	// Invert the option switches - they're all active low normally and pulled up if open
	i ^= (OPTION_A_APPROACH_LIGHTING | OPTION_B_FOUR_ASPECT | OPTION_C_SEARCHLIGHT_MODE | OPTION_D_RESERVED | OPTION_E_RESERVED);

	i &= ~(SWITCH_SW3_IN | SWITCH_SW4_IN); // These are SW_A and SW_B.  We're going to reuse their bits for other stuff.
	i |= ((PINA & 0x02)?OPTION_COMMON_ANODE:0);

	*retval = i;
	return true;
}

bool getMSS(uint8_t* retval, bool mss_v2)
{
	uint8_t i = 0;

	if (!readByte(TCA9555_ADDR_000, TCA9555_GPIN1, &i))
		return false;

	// If we're set for MSS v2, the approach diverging lines are active low (after inversion)
	//  so we need to re-invert them here so that if the bit is on, the line is active
	if (mss_v2)
		i ^= (MSS_B_AD_IN | MSS_A_AD_IN);

	*retval = i;
	return true;
}

void lampTest()
{
	uint8_t mask = 0x01;
	uint8_t i = 0;
	for(i=0; i<6; i++)
	{
		writeByte(TCA9555_ADDR_001, TCA9555_GPOUT1, mask);

		mask <<= 1;
		wdt_reset();
		_delay_ms(100);
	}

	writeByte(TCA9555_ADDR_001, TCA9555_GPOUT1, 0);
	mask = 0x01;

	for(i=0; i<8; i++)
	{
		writeByte(TCA9555_ADDR_001, TCA9555_GPOUT0, mask);
		mask <<= 1;
		wdt_reset();
		_delay_ms(100);
	}
	
	writeByte(TCA9555_ADDR_001, TCA9555_GPOUT0, 0);
	writeByte(TCA9555_ADDR_001, TCA9555_GPOUT1, 0);
}

bool testSignalEEPROMValid()
{
	for (uint8_t* head=EEPROM_SIGNAL_AU_BASE; head<=EEPROM_SIGNAL_BL_BASE; head += 0x08)
	{
		for (uint8_t ind=INDICATION_STOP; ind<=INDICATION_CLEAR; ind++)
		{
			if (eeprom_read_byte(head + ind) >= ASPECT_END)
				return false;
		}
	}
	return true;

}

void initializeSignalEEPROM()
{
	eeprom_write_byte(EEPROM_SIGNAL_AU_BASE + INDICATION_STOP,                  ASPECT_RED);
	eeprom_write_byte(EEPROM_SIGNAL_AU_BASE + INDICATION_APPROACH,              ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_AU_BASE + INDICATION_ADVANCE_APPROACH,      ASPECT_FL_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_AU_BASE + INDICATION_APPROACH_DIVERGING_AA, ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_AU_BASE + INDICATION_APPROACH_DIVERGING,    ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_AU_BASE + INDICATION_CLEAR,                 ASPECT_GREEN);

	eeprom_write_byte(EEPROM_SIGNAL_AL_BASE + INDICATION_STOP,                  ASPECT_RED);
	eeprom_write_byte(EEPROM_SIGNAL_AL_BASE + INDICATION_APPROACH,              ASPECT_RED);
	eeprom_write_byte(EEPROM_SIGNAL_AL_BASE + INDICATION_ADVANCE_APPROACH,      ASPECT_RED);
	eeprom_write_byte(EEPROM_SIGNAL_AL_BASE + INDICATION_APPROACH_DIVERGING_AA, ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_AL_BASE + INDICATION_APPROACH_DIVERGING,    ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_AL_BASE + INDICATION_CLEAR,                 ASPECT_RED);

	eeprom_write_byte(EEPROM_SIGNAL_BU_BASE + INDICATION_STOP,                  ASPECT_RED);
	eeprom_write_byte(EEPROM_SIGNAL_BU_BASE + INDICATION_APPROACH,              ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_BU_BASE + INDICATION_ADVANCE_APPROACH,      ASPECT_FL_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_BU_BASE + INDICATION_APPROACH_DIVERGING_AA, ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_BU_BASE + INDICATION_APPROACH_DIVERGING,    ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_BU_BASE + INDICATION_CLEAR,                 ASPECT_GREEN);

	eeprom_write_byte(EEPROM_SIGNAL_BL_BASE + INDICATION_STOP,                  ASPECT_RED);
	eeprom_write_byte(EEPROM_SIGNAL_BL_BASE + INDICATION_APPROACH,              ASPECT_RED);
	eeprom_write_byte(EEPROM_SIGNAL_BL_BASE + INDICATION_ADVANCE_APPROACH,      ASPECT_RED);
	eeprom_write_byte(EEPROM_SIGNAL_BL_BASE + INDICATION_APPROACH_DIVERGING_AA, ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_BL_BASE + INDICATION_APPROACH_DIVERGING,    ASPECT_YELLOW);
	eeprom_write_byte(EEPROM_SIGNAL_BL_BASE + INDICATION_CLEAR,                 ASPECT_RED);
}

typedef enum 
{
	CONFIG_OFF                  = 0,
	CONFIG_START                = 1,
	CONFIG_SHUTDOWN             = 2,
	CONFIG_RUN                  = 3,
	CONFIG_END
} ConfigState_t;

#define SIGNAL_A  0x00
#define SIGNAL_B  0x01


void showConfigState(uint8_t signal, MSSPortIndication_t ind, SignalAspect_t upper, SignalAspect_t lower, bool flasher)
{
	uint8_t configLEDOutput = 0;
	uint8_t mockSignalLEDOutput = 0;
	
	if (SIGNAL_A == signal)
		configLEDOutput = CONF_LED_SIGNAL_A;
	else if (SIGNAL_B == signal)
		configLEDOutput = CONF_LED_SIGNAL_B;
	
	switch(ind)
	{
		case INDICATION_STOP:
			configLEDOutput |= CONF_LED_ASPECT_S;
			break;

		case INDICATION_APPROACH:
			configLEDOutput |= CONF_LED_ASPECT_A;
			break;

		case INDICATION_ADVANCE_APPROACH:
			configLEDOutput |= CONF_LED_ASPECT_AA;
			break;
			
		case INDICATION_APPROACH_DIVERGING_AA:
			configLEDOutput |= CONF_LED_ASPECT_AD_AA;
			break;

		case INDICATION_APPROACH_DIVERGING:
			configLEDOutput |= CONF_LED_ASPECT_AD;
			break;

		case INDICATION_CLEAR:
			configLEDOutput |= CONF_LED_ASPECT_CLR;
			break;
			
		default:
			break;
	}

	writeByte(TCA9555_ADDR_001, TCA9555_GPOUT0, configLEDOutput);

	switch(upper)
	{
		case ASPECT_RED:
			mockSignalLEDOutput |= CONF_LED_UPPER_RED;
			break;
			
		case ASPECT_YELLOW:	
			mockSignalLEDOutput |= CONF_LED_UPPER_YELLOW;		
			break;		

		case ASPECT_GREEN:	
			mockSignalLEDOutput |= CONF_LED_UPPER_GREEN;		
			break;		

		case ASPECT_FL_RED:
			if (flasher)
				mockSignalLEDOutput |= CONF_LED_UPPER_RED;
			break;
			
		case ASPECT_FL_YELLOW:	
			if (flasher)
				mockSignalLEDOutput |= CONF_LED_UPPER_YELLOW;		
			break;		

		case ASPECT_FL_GREEN:	
			if (flasher)
				mockSignalLEDOutput |= CONF_LED_UPPER_GREEN;		
			break;		

		case ASPECT_OFF:
		default:
			break;
	}

	switch(lower)
	{
		case ASPECT_RED:
			mockSignalLEDOutput |= CONF_LED_LOWER_RED;
			break;
			
		case ASPECT_YELLOW:	
			mockSignalLEDOutput |= CONF_LED_LOWER_YELLOW;		
			break;		

		case ASPECT_GREEN:	
			mockSignalLEDOutput |= CONF_LED_LOWER_GREEN;		
			break;		

		case ASPECT_FL_RED:
			if (flasher)
				mockSignalLEDOutput |= CONF_LED_LOWER_RED;
			break;
			
		case ASPECT_FL_YELLOW:	
			if (flasher)
				mockSignalLEDOutput |= CONF_LED_LOWER_YELLOW;		
			break;		

		case ASPECT_FL_GREEN:	
			if (flasher)
				mockSignalLEDOutput |= CONF_LED_LOWER_GREEN;		
			break;		

		case ASPECT_OFF:
		default:
			break;
	}
	writeByte(TCA9555_ADDR_001, TCA9555_GPOUT1, mockSignalLEDOutput);

}



int main(void)
{
	DebounceState8_t optionsDebouncer;
	DebounceState8_t mssDebouncer;
	DebounceState8_t switchDebouncer;  // Used for the pushbuttons
	
	uint32_t lastReadTime = 0xf0000000;
	uint32_t lastSwitchTime = 0xf0000000;
	
	uint32_t currentTime = 0;
	uint8_t initValue=0, i=0;
	
	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	// PORT A
	//  PA7 - Output - A Upper Signal - RED
	//  PA6 - Output - A Upper Signal - YELLOW
	//  PA5 - Output - A Upper Signal - GREEN
	//  PA4 - Output - A Lower Signal - RED
	//  PA3 - Output - A Lower Signal - YELLOW
	//  PA2 - I2C    - SCL
	//  PA1 - Input  - Common Connection Sense
	//  PA0 - I2C    - SDA

	// PORT B
	//  PB7 - n/a    - /RESET (not I/O pin)
	//  PB6 - Output - B Upper Signal - RED
	//  PB5 - Output - B Upper Signal - YELLOW
	//  PB5 - Output - B Upper Signal - GREEN
	//  PB3 - Output - B Lower Signal - RED
	//  PB2 - Output - B Lower Signal - YELLOW
	//  PB1 - Output - B Lower Signal - GREEN
	//  PB0 - Output - A Lower Signal - GREEN

	PORTA = 0b11111101;
	DDRA  = 0b11111100;

	PORTB = 0b11111111;
	DDRB  = 0b01111111;

	initializeTimer();

	signalHeadInitialize(&signalAU);
	signalHeadInitialize(&signalAL);
	signalHeadInitialize(&signalBU);
	signalHeadInitialize(&signalBL);

	signalHeadAspectSet(&signalAU, ASPECT_OFF);
	signalHeadAspectSet(&signalAL, ASPECT_OFF);
	signalHeadAspectSet(&signalBU, ASPECT_OFF);
	signalHeadAspectSet(&signalBL, ASPECT_OFF);

	wdt_reset();

	// Set TCA9555 direction registers
	writeByte(TCA9555_ADDR_000, TCA9555_GPDDR0, 0b11111111);
	writeByte(TCA9555_ADDR_000, TCA9555_GPDDR1, 0b11111111);
	writeByte(TCA9555_ADDR_000, TCA9555_GPOUT0, 0b00000000);

	writeByte(TCA9555_ADDR_001, TCA9555_GPDDR0, 0b00000000);
	writeByte(TCA9555_ADDR_001, TCA9555_GPDDR1, 0b11000000);
	writeByte(TCA9555_ADDR_001, TCA9555_GPOUT0, 0b00000000);
	writeByte(TCA9555_ADDR_001, TCA9555_GPOUT1, 0b00000000);

	// Now that the IO expander and such as had a little time to settle
	//  go get the initial values for options and MSS.  This should avoid
	//  most of the initial bouncing around as the module starts up.

	initValue = OPTION_COMMON_ANODE | OPTION_B_FOUR_ASPECT; // Default is common anode, four aspect

	if (getOptions(&i))
		initValue = i;

	initDebounceState8(&optionsDebouncer, initValue);

	// Convert global option bits to signal head option bits
	if (initValue & OPTION_C_SEARCHLIGHT_MODE)
		signalHeadOptions |= SIGNAL_OPTION_SEARCHLIGHT;
	else 
		signalHeadOptions &= ~SIGNAL_OPTION_SEARCHLIGHT;

	if (initValue & OPTION_COMMON_ANODE)
		signalHeadOptions |= SIGNAL_OPTION_COMMON_ANODE;
	else 
		signalHeadOptions &= ~SIGNAL_OPTION_COMMON_ANODE;

	initValue = 0;
	if (getMSS(&i, getDebouncedState(&optionsDebouncer) & OPTION_MSS_V2))
		initValue = i;
	initDebounceState8(&mssDebouncer, initValue);

	initValue = 0;
	if (getSwitches(&i))
		initValue = i;
	initDebounceState8(&switchDebouncer, initValue);

	if (!testSignalEEPROMValid())
		initializeSignalEEPROM();


	sei();

	// Do an initial LED dance to verify that everything's connected
	lampTest();
	
	ConfigState_t cState = CONFIG_OFF;

	while(1)
	{
		wdt_reset();

		currentTime = getMillis();

		if (((uint32_t)currentTime - lastSwitchTime) > SWITCH_UPDATE_TIME_MS)
		{
			uint8_t switchesPressed = 0;
			static uint8_t signal=0;
			static MSSPortIndication_t ind = 0;
			static SignalAspect_t upperMockAspect = ASPECT_OFF;
			static SignalAspect_t lowerMockAspect = ASPECT_OFF;
			
			lastSwitchTime = currentTime;
			if (getSwitches(&i))
				switchesPressed = debounce8(i, &switchDebouncer);

			if (CONFIG_OFF == cState && getDebouncedState(&optionsDebouncer) & OPTION_E_RESERVED)
			{
				if ((0x0F & (~getDebouncedState(&switchDebouncer))) == (SWITCHMASK_LEFT | SWITCHMASK_RIGHT))
					initializeSignalEEPROM();
				cState = CONFIG_START;
			}
			
			if (CONFIG_OFF != cState && !(getDebouncedState(&optionsDebouncer) & OPTION_E_RESERVED))
				cState = CONFIG_SHUTDOWN;

			if (CONFIG_OFF != cState)
				showConfigState(signal, ind, upperMockAspect, lowerMockAspect, flasher);

			switch(cState)
			{
				case CONFIG_OFF:
					break;
				
				case CONFIG_START:
					signal = SIGNAL_A;
					ind = INDICATION_STOP;
					cState = CONFIG_RUN;
					break;
			
				case CONFIG_RUN:
					if (switchesPressed & SWITCHMASK_LEFT)
						signal = (signal + 1) & 0x01;

					if (switchesPressed & SWITCHMASK_RIGHT)
						if (++ind >= INDICATION_END)
							ind = INDICATION_STOP;

					if (SIGNAL_A == signal)
					{
						upperMockAspect = eeprom_read_byte(EEPROM_SIGNAL_AU_BASE + ind);
						lowerMockAspect = eeprom_read_byte(EEPROM_SIGNAL_AL_BASE + ind);
					} else if (SIGNAL_B == signal) {
						upperMockAspect = eeprom_read_byte(EEPROM_SIGNAL_BU_BASE + ind);
						lowerMockAspect = eeprom_read_byte(EEPROM_SIGNAL_BL_BASE + ind);					
					}

					if (switchesPressed & SWITCHMASK_UPPER_HEAD)
						if (++upperMockAspect > ASPECT_FL_RED)
							upperMockAspect = ASPECT_OFF;					

					if (switchesPressed & SWITCHMASK_LOWER_HEAD)
						if (++lowerMockAspect > ASPECT_FL_RED)
							lowerMockAspect = ASPECT_OFF;							

					if (SIGNAL_A == signal)
					{
						eeprom_update_byte(EEPROM_SIGNAL_AU_BASE + ind, upperMockAspect);
						eeprom_update_byte(EEPROM_SIGNAL_AL_BASE + ind, lowerMockAspect);
					} else if (SIGNAL_B == signal) {
						eeprom_update_byte(EEPROM_SIGNAL_BU_BASE + ind, upperMockAspect);
						eeprom_update_byte(EEPROM_SIGNAL_BL_BASE + ind, lowerMockAspect);					
					}
					break;
				
				case CONFIG_SHUTDOWN:
					writeByte(TCA9555_ADDR_001, TCA9555_GPOUT0, 0);
					writeByte(TCA9555_ADDR_001, TCA9555_GPOUT1, 0);
					cState = CONFIG_OFF;
					break;
			
				default:
					cState = CONFIG_SHUTDOWN;
					break;			
			}
			
				
		}


		// We really don't need or want updates very often
		//  50mS or so should be fine.  
		if (((uint32_t)currentTime - lastReadTime) > LOOP_UPDATE_TIME_MS)
		{
			uint8_t optionJumpers = 0;
			uint8_t mssInputs = 0;
			lastReadTime = currentTime;

			if (getOptions(&i))
				debounce8(i, &optionsDebouncer);
			
			if (getMSS(&i, getDebouncedState(&optionsDebouncer) & OPTION_MSS_V2))
				debounce8(i, &mssDebouncer);

			// Read state of MSS bus ports coming in
			mssInputs = getDebouncedState(&mssDebouncer);

			// Read state of option jumpers
			optionJumpers = getDebouncedState(&optionsDebouncer);

			// If you want to fake the optionJumper settings for testing, do it here
			// optionJumpers |= OPTION_B_FOUR_ASPECT | OPTION_C_SEARCHLIGHT_MODE | OPTION_D_LIMIT_DIVERGING | OPTION_A_APPROACH_LIGHTING;

			// Convert global option bits to signal head option bits
			if (optionJumpers & OPTION_C_SEARCHLIGHT_MODE)
				signalHeadOptions |= SIGNAL_OPTION_SEARCHLIGHT;
			else 
				signalHeadOptions &= ~SIGNAL_OPTION_SEARCHLIGHT;

			if (optionJumpers & OPTION_COMMON_ANODE)
				signalHeadOptions |= SIGNAL_OPTION_COMMON_ANODE;
			else 
				signalHeadOptions &= ~SIGNAL_OPTION_COMMON_ANODE;

			calculateAspects(mssInputs, optionJumpers);
		}
	}
}



void calculateAspects(uint8_t mssInputs, uint8_t optionJumpers)
{
	SignalAspect_t aspectAU = ASPECT_RED;
	SignalAspect_t aspectAL = ASPECT_RED;
	SignalAspect_t aspectBU = ASPECT_RED;
	SignalAspect_t aspectBL = ASPECT_RED;

	MSSPortIndication_t portA = INDICATION_STOP;
	MSSPortIndication_t portB = INDICATION_STOP;
	
	// Calculate signal aspects directly, since we don't have neatly-terminated
	//  MSS busses but rather have to glean states from combinations of things.

	// Two steps.  Convert wires to indication state, convert indication state to aspects
	portA = mssPortWiresToIndication(mssInputs & MSS_A_S, mssInputs & MSS_A_A_IN, mssInputs & MSS_A_AA_IN, mssInputs & MSS_A_AD_IN);
	portB = mssPortWiresToIndication(mssInputs & MSS_B_S, mssInputs & MSS_B_A_IN, mssInputs & MSS_B_AA_IN, mssInputs & MSS_B_AD_IN);

	
	aspectAU = eeprom_read_byte(EEPROM_SIGNAL_AU_BASE + portB);
	aspectAL = eeprom_read_byte(EEPROM_SIGNAL_AL_BASE + portB);
	aspectBU = eeprom_read_byte(EEPROM_SIGNAL_BU_BASE + portA);
	aspectBL = eeprom_read_byte(EEPROM_SIGNAL_BL_BASE + portA);


	// Handle approach lighting.  If we're approach lit, turn everything off unless there's
	//  something in an adjacent block
	if (optionJumpers & OPTION_A_APPROACH_LIGHTING)
	{
		if (!(mssInputs & (MSS_A_S | MSS_B_S)))
		{
			// If nothing is in any of the adjacent blocks, turn signals off
			aspectAL = aspectAU = aspectBL = aspectBU = ASPECT_OFF;
		}
	}

	signalHeadAspectSet(&signalAU, aspectAU);
	signalHeadAspectSet(&signalAL, aspectAL);
	signalHeadAspectSet(&signalBU, aspectBU);
	signalHeadAspectSet(&signalBL, aspectBL);
}




