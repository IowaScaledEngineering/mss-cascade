#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "mss-cascade.h"

#define SIGA1_RED_PD  PD3
#define SIGA1_YEL_PD  PD2
#define SIGA1_GRN_PD  PD1

void setSignalA1(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t mask = ((red?_BV(SIGA1_RED_PD):0) | (yellow?_BV(SIGA1_YEL_PD):0) | (green?_BV(SIGA1_GRN_PD):0));
	if (ca)
		PORTD = (PORTD | (_BV(SIGA1_RED_PD) | _BV(SIGA1_YEL_PD) | _BV(SIGA1_GRN_PD))) & ~mask;
	else
		PORTD = (PORTD & ~(_BV(SIGA1_RED_PD) | _BV(SIGA1_YEL_PD) | _BV(SIGA1_GRN_PD))) | mask;
}


#define SIGA2_RED_PD  PD6
#define SIGA2_YEL_PD  PD5
#define SIGA2_GRN_PD  PD4

void setSignalA2(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t mask = ((red?_BV(SIGA2_RED_PD):0) | (yellow?_BV(SIGA2_YEL_PD):0) | (green?_BV(SIGA2_GRN_PD):0));
	if (ca)
		PORTD = (PORTD | (_BV(SIGA2_RED_PD) | _BV(SIGA2_YEL_PD) | _BV(SIGA2_GRN_PD))) & ~mask;
	else
		PORTD = (PORTD & ~(_BV(SIGA2_RED_PD) | _BV(SIGA2_YEL_PD) | _BV(SIGA2_GRN_PD))) | mask;
}

#define SIGB1_RED_PD  PD7
#define SIGB1_YEL_PE  PE6
#define SIGB1_GRN_PE  PE3


void setSignalB1(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t maskd = (red?_BV(SIGB1_RED_PD):0);
	uint8_t maske = (yellow?_BV(SIGB1_YEL_PE):0) | (green?_BV(SIGB1_GRN_PE):0);
	
	if (ca)
	{
		PORTD = (PORTD | (_BV(SIGB1_RED_PD))) & ~maskd;
		PORTE = (PORTE | (_BV(SIGB1_YEL_PE) | _BV(SIGB1_GRN_PE))) & ~maske;
	}
	else
	{
		PORTD = (PORTD & ~(_BV(SIGB1_RED_PD))) | maskd;
		PORTE = (PORTE & ~(_BV(SIGB1_YEL_PE) | _BV(SIGB1_GRN_PE))) | maske;
	}
}

#define SIGB2_RED_PF  PF7
#define SIGB2_YEL_PF  PF6
#define SIGB2_GRN_PE  PE7


void setSignalB2(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t maske = (green?_BV(SIGB2_GRN_PE):0);
	uint8_t maskf = (yellow?_BV(SIGB2_YEL_PF):0) | (red?_BV(SIGB2_RED_PF):0);
	
	if (ca)
	{
		PORTE = (PORTE | (_BV(SIGB2_GRN_PE))) & ~maske;
		PORTF = (PORTF | (_BV(SIGB2_YEL_PF) | _BV(SIGB2_RED_PF))) & ~maskf;
	}
	else
	{
		PORTE = (PORTE & ~(_BV(SIGB2_GRN_PE))) | maske;
		PORTF = (PORTF & ~(_BV(SIGB2_YEL_PF) | _BV(SIGB2_RED_PF))) | maskf;
	}
}

void IndicationToOutputs(SetSignal setSignal, SignalHeadType signalHeadType, SignalAspect signalAspect, uint8_t blinkOn, uint8_t slice, uint8_t trim)
{
	uint8_t commonAnode = 0;
	uint8_t redStop = 32;
	uint8_t greenStop = 32;

	if (trim <= 15)
		redStop -= (15-trim);
	else
		greenStop -= (trim - 15);




	if (blinkOn)
		blinkOn = 1;

	switch(signalHeadType)
	{
		case SIGNAL_3LIGHT_CA:
			commonAnode = 1;
		case SIGNAL_3LIGHT_CC:
			switch(signalAspect)
			{
				case ASPECT_RED:
					setSignal(1,0,0, commonAnode);
					break;
				case ASPECT_YELLOW:
					setSignal(0,1,0, commonAnode);				
					break;
				case ASPECT_GREEN:
					setSignal(0,0,1, commonAnode);				
					break;
				case ASPECT_FL_RED:
					setSignal(blinkOn,0,0, commonAnode);
					break;
				case ASPECT_FL_YELLOW:
					setSignal(0,blinkOn,0, commonAnode);				
					break;
				case ASPECT_FL_GREEN:
					setSignal(0,0,blinkOn, commonAnode);				
					break;
				case ASPECT_OFF:
				default:
					setSignal(0,0,0, commonAnode);
					break;
			}
			break;

		case SIGNAL_3WIRE_RG_CA:
			commonAnode = 1;
		case SIGNAL_3WIRE_RG_CC:
			switch(signalAspect)
			{
				case ASPECT_RED:
					setSignal(1,0,0, commonAnode);
					break;
				case ASPECT_YELLOW:
					setSignal((slice >= redStop)?0:1, 0, (slice >= greenStop)?0:1, commonAnode);
					break;
				case ASPECT_GREEN:
					setSignal(0,0,1, commonAnode);				
					break;
				case ASPECT_FL_RED:
					setSignal((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
					break;
				case ASPECT_FL_YELLOW:
					setSignal((slice >= redStop)?0:(blinkOn),0,(slice >= greenStop)?0:(blinkOn), commonAnode);				
					break;
				case ASPECT_FL_GREEN:
					setSignal(0,0,blinkOn, commonAnode);
					break;
				case ASPECT_OFF:
				default:
					setSignal(0,0,0, commonAnode);
					break;
			}
			break;
		
		case SIGNAL_2WIRE_RG:
			switch(signalAspect)
			{
				case ASPECT_RED:
					setSignal(1,0,0, 0);
					break;
				case ASPECT_YELLOW:
					setSignal((slice < redStop && 0 == (slice % 2))?1:0, 0, (slice < greenStop && (slice % 2))?1:0, 0);
					break;
				case ASPECT_GREEN:
					setSignal(0,0,1, 0);				
					break;
				case ASPECT_FL_RED:
					setSignal(blinkOn,0,0, 0);
					break;
				case ASPECT_FL_YELLOW:
					setSignal((slice < redStop && 0 == (slice % 2))?(blinkOn):0, 0, (slice < greenStop && (slice % 2))?(blinkOn):0, 0);
					break;
				case ASPECT_FL_GREEN:
					setSignal(0,0,blinkOn, 0);
					break;
				case ASPECT_OFF:
				default:
					setSignal(0,0,0, 0);
					break;
			}
			break;

		default:
			setSignal(0,0,0,0);
			break;		
	}


}
