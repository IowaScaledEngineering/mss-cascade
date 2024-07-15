#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "mss-cascade.h"

uint16_t configSwitches = 0;

uint8_t signalsApproachLit()
{
	return (configSwitches & _BV(CONFIG_APPROACH_LIT))?1:0;
}

uint8_t signalAIsSingleHead()
{
	return (configSwitches & _BV(CONFIG_SIGNAL_A_DUAL))?0:1;
}

uint8_t signalAIsDualHead()
{
	return (signalAIsSingleHead())?0:1;
}

uint8_t signalBIsSingleHead()
{
	return (configSwitches & _BV(CONFIG_SIGNAL_B_DUAL))?0:1;
}

uint8_t signalBIsDualHead()
{
	return (signalBIsSingleHead())?0:1;
}

uint8_t signalAAspectConfiguration()
{
	return (signalAIsDualHead()?4:0) + ((configSwitches & (_BV(CONFIG_SIGNAL_A_ASPECTS0) | _BV(CONFIG_SIGNAL_A_ASPECTS1)))>>CONFIG_SIGNAL_A_ASPECTS0);
}

uint8_t signalBAspectConfiguration()
{
	return (signalBIsDualHead()?4:0) + ((configSwitches & (_BV(CONFIG_SIGNAL_B_ASPECTS0) | _BV(CONFIG_SIGNAL_B_ASPECTS1)))>>CONFIG_SIGNAL_B_ASPECTS0);
}

void signalConfigToSignalType()
{
	switch(configSwitches & 0x07)
	{
		case 0:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_3LIGHT_CA;
			break;
			
		case 1:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_3LIGHT_CC;
			break;

		case 2:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_3WIRE_RG_CA;
			break;		

		case 3:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_3WIRE_RG_CC;
			break;		

		case 4:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_2WIRE_RG;
			break;		
		
		default:	
			signalHeadTypeA1 = signalHeadTypeA2 = signalHeadTypeB1 = signalHeadTypeB2 = SIGNAL_UNKNOWN;
			break;
	}

	if (SIGNAL_UNKNOWN == signalHeadTypeA1)
		return;
	// This is common for all cases except the unknown, so put it here
	signalHeadTypeA2 = signalAIsSingleHead()?SIGNAL_3LIGHT_CA:signalHeadTypeA1;
	signalHeadTypeB2 = signalBIsSingleHead()?SIGNAL_3LIGHT_CA:signalHeadTypeB1;
}

void debounceConfig()
{
	static uint16_t clock_A=0, clock_B=0;
	uint16_t rawInput = ~((uint16_t)PINC | (((uint16_t)(PING & _BV(PG2)))<<6) | (((uint16_t)(PINA & _BV(PA7)))<<2));
	uint16_t delta = rawInput ^ configSwitches;
	uint16_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;
	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.
	changes = ~((~delta) | clock_A | clock_B);
	configSwitches ^= changes;	
}


