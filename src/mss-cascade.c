#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "mss-cascade.h"

volatile uint8_t eventFlags = 0;

void initialize10HzTimer()
{
	// Set up timer 1 for 10Hz interrupts
	TCNT1 = 0;
	OCR1A = 0x0C34;
	TCCR1A = 0;
	TCCR1B = _BV(WGM12) | _BV(CS12);
	TCCR1C = 0;
	TIFR1 |= _BV(OCF1A);
	TIMSK1 |= _BV(OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
	static uint8_t cntr = 0;
	eventFlags |= EVENT_DO_ADC_RUN;

	if (++cntr >= 5)
	{
		eventFlags ^= EVENT_1HZ_BLINK;
		cntr = 0;
	}
}


void initialize4000HzTimer()
{
	// Set up timer 2 for 4000Hz interrupts
	TCNT2 = 0;
	OCR2A = 0x7E;
	TCCR2A = _BV(WGM21) | _BV(CS21) | _BV(CS20);
	TIFR2 |= _BV(OCF2A);
	TIMSK2 |= _BV(OCIE2A);
}

SignalHeadType signalHeadTypeA1 = SIGNAL_3LIGHT_CA;
SignalHeadType signalHeadTypeA2 = SIGNAL_3LIGHT_CA;
SignalHeadType signalHeadTypeB1 = 0;
SignalHeadType signalHeadTypeB2 = 0;


SignalAspect signalAspectA1 = ASPECT_FL_YELLOW;
SignalAspect signalAspectA2 = ASPECT_OFF;
SignalAspect signalAspectB1 = ASPECT_OFF;
SignalAspect signalAspectB2 = ASPECT_OFF;


/*
	// Pin Assignments for PORTD/DDRD
	//  PD0 - N/C
	//  PD1 - A Green 1  (out)
	//  PD2 - A Yellow 1 (out)
	//  PD3 - A Red 1    (out)
	//  PD4 - A Green 2  (out)
	//  PD5 - A Yellow 2 (out)
	//  PD6 - A Red 2    (out)
	//  PD7 - B Red 1    (out)
	DDRD  = 0b11111111;
	PORTD = 0b00000000;

	// Pin Assignments for PORTE/DDRE
	//  PE0 - RX (N/C)
	//  PE1 - TX (N/C)
	//  PE2 - N/C
	//  PE3 - B Green 1  (out)
	//  PE4 - SCL (in)
	//  PE5 - SDA (in)
	//  PE6 - B Yellow 1 (out)
	//  PE7 - B Green 2  (out)
	DDRE  = 0b11011111;
	PORTE = 0b00110000;
	
	// Pin Assignments for PORTE/DDRE
	//  PF0 - Analog 0 - Setpoint 1
	//  PF1 - Analog 1 - Setpoint 2
	//  PF2 - Analog 2 - Detector 1
	//  PF3 - Analog 3 - Detector 2
	//  PF4 - Analog 4 - A Red/Green balance
	//  PF5 - Analog 5 - B Red/Green balance
	//  PF6 - B Yellow 2 (out)
	//  PF7 - B Red 2    (out)
	DDRF  = 0b11000000;
	PORTF = 0b00000000;
*/

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


ISR(TIMER2_COMP_vect)
{
	uint8_t commonAnode = 0;
	uint8_t redStop = 32;
	uint8_t greenStop = 32;
	
	// This is where signal output generation is handled.  

	static uint8_t trim = 0;

	if (++trim >= 32)
		trim = 0;
		
	if (trim

	switch(signalHeadTypeA1)
	{
		case SIGNAL_3LIGHT_CA:
			commonAnode = 1;
		case SIGNAL_3LIGHT_CC:
			switch(signalAspectA1)
			{
				case ASPECT_RED:
					setSignalA1(1,0,0, commonAnode);
					break;
				case ASPECT_YELLOW:
					setSignalA1(0,1,0, commonAnode);				
					break;
				case ASPECT_GREEN:
					setSignalA1(0,0,1, commonAnode);				
					break;
				case ASPECT_FL_RED:
					setSignalA1((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
					break;
				case ASPECT_FL_YELLOW:
					setSignalA1(0,(eventFlags & EVENT_1HZ_BLINK),0, commonAnode);				
					break;
				case ASPECT_FL_GREEN:
					setSignalA1(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);				
					break;
				case ASPECT_OFF:
				default:
					setSignalA1(0,0,0, commonAnode);
					break;
			}
			break;

		case SIGNAL_3WIRE_RG_CA:
			commonAnode = 1;
		case SIGNAL_3WIRE_RG_CC:
			switch(signalAspectA1)
			{
				case ASPECT_RED:
					setSignalA1(1,0,0, commonAnode);
					break;
				case ASPECT_YELLOW:
					setSignalA1(1,0,1, commonAnode);
					break;
				case ASPECT_GREEN:
					setSignalA1(0,0,1, commonAnode);				
					break;
				case ASPECT_FL_RED:
					setSignalA1((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
					break;
				case ASPECT_FL_YELLOW:
					setSignalA1((eventFlags & EVENT_1HZ_BLINK),0,(eventFlags & EVENT_1HZ_BLINK), commonAnode);				
					break;
				case ASPECT_FL_GREEN:
					setSignalA1(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);
					break;
				case ASPECT_OFF:
				default:
					setSignalA1(0,0,0, commonAnode);
					break;
			}
			break;
		


		default:
			setSignalA1(0,0,0, commonAnode);
			break;		
	}

	commonAnode = 0;

	switch(signalHeadTypeA2)
	{
		case SIGNAL_3LIGHT_CA:
			commonAnode = 1;		
		case SIGNAL_3LIGHT_CC:
			switch(signalAspectA2)
			{
				case ASPECT_RED:
					setSignalA2(1,0,0, commonAnode);
					break;
				case ASPECT_YELLOW:
					setSignalA2(0,1,0, commonAnode);				
					break;
				case ASPECT_GREEN:
					setSignalA2(0,0,1, commonAnode);				
					break;
				case ASPECT_FL_RED:
					setSignalA2((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
					break;
				case ASPECT_FL_YELLOW:
					setSignalA2(0,(eventFlags & EVENT_1HZ_BLINK),0, commonAnode);				
					break;
				case ASPECT_FL_GREEN:
					setSignalA2(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);				
					break;
				case ASPECT_OFF:
				default:
					setSignalA2(0,0,0, commonAnode);
					break;			

			}		
			break;

		default:
			setSignalA2(0,0,0, commonAnode);
			break;			
		
	}

	commonAnode = 0;
	// This is where signal output generation is handled.  

	switch(signalHeadTypeB1)
	{
		case SIGNAL_3LIGHT_CA:
			commonAnode = 1;
		case SIGNAL_3LIGHT_CC:
			switch(signalAspectB1)
			{
				case ASPECT_RED:
					setSignalB1(1,0,0, commonAnode);
					break;
				case ASPECT_YELLOW:
					setSignalB1(0,1,0, commonAnode);				
					break;
				case ASPECT_GREEN:
					setSignalB1(0,0,1, commonAnode);				
					break;
				case ASPECT_FL_RED:
					setSignalB1((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
					break;
				case ASPECT_FL_YELLOW:
					setSignalB1(0,(eventFlags & EVENT_1HZ_BLINK),0, commonAnode);				
					break;
				case ASPECT_FL_GREEN:
					setSignalB1(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);				
					break;
				case ASPECT_OFF:
				default:
					setSignalB1(0,0,0, commonAnode);
					break;
			}
			break;

		case SIGNAL_3WIRE_RG_CA:
			commonAnode = 1;
		case SIGNAL_3WIRE_RG_CC:
			switch(signalAspectB1)
			{
				case ASPECT_RED:
					setSignalB1(1,0,0, commonAnode);
					break;
				case ASPECT_YELLOW:
					setSignalB1(1,0,1, commonAnode);
					break;
				case ASPECT_GREEN:
					setSignalB1(0,0,1, commonAnode);				
					break;
				case ASPECT_FL_RED:
					setSignalB1((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
					break;
				case ASPECT_FL_YELLOW:
					setSignalB1((eventFlags & EVENT_1HZ_BLINK),0,(eventFlags & EVENT_1HZ_BLINK), commonAnode);				
					break;
				case ASPECT_FL_GREEN:
					setSignalB1(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);
					break;
				case ASPECT_OFF:
				default:
					setSignalA1(0,0,0, commonAnode);
					break;
			}
			break;


		default:
			setSignalB1(0,0,0, commonAnode);
			break;		
	}

	commonAnode = 0;
	// This is where signal output generation is handled.  

	switch(signalHeadTypeB2)
	{
		case SIGNAL_3LIGHT_CA:
			commonAnode = 1;
		case SIGNAL_3LIGHT_CC:
			switch(signalAspectB2)
			{
				case ASPECT_RED:
					setSignalB2(1,0,0, commonAnode);
					break;
				case ASPECT_YELLOW:
					setSignalB2(0,1,0, commonAnode);				
					break;
				case ASPECT_GREEN:
					setSignalB2(0,0,1, commonAnode);				
					break;
				case ASPECT_FL_RED:
					setSignalB2((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
					break;
				case ASPECT_FL_YELLOW:
					setSignalB2(0,(eventFlags & EVENT_1HZ_BLINK),0, commonAnode);				
					break;
				case ASPECT_FL_GREEN:
					setSignalB2(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);				
					break;
				case ASPECT_OFF:
				default:
					setSignalB2(0,0,0, commonAnode);
					break;
			}
			break;

		default:
			setSignalB2(0,0,0, commonAnode);
			break;		
	}

}

uint16_t configSwitches = 0;

inline uint8_t signalsApproachLit()
{
	return (configSwitches & _BV(CONFIG_APPROACH_LIT))?1:0;
}

/*
Bits:
 0/1/2 - Signal Type
 3 - A dual head
 4 - B dual head
 8 - 
 9 - Approach lit


210 - Signal type
000 - 3 wire common anode
001 - 3 wire common cathode
010 - 3 wire R/G CA
011 - 3 wire R/G CC
100 - 2 wire R/G

*/


uint8_t signalAIsSingleHead()
{
	return (configSwitches & _BV(CONFIG_SIGNAL_A_DUAL))?1:0;
}

uint8_t signalAIsDualHead()
{
	return (configSwitches & _BV(CONFIG_SIGNAL_A_DUAL))?0:1;
}

uint8_t signalBIsSingleHead()
{
	return (configSwitches & _BV(CONFIG_SIGNAL_B_DUAL))?1:0;
}

uint8_t signalBIsDualHead()
{
	return (configSwitches & _BV(CONFIG_SIGNAL_B_DUAL))?0:1;
}


void signalConfigToSignalType()
{
	switch(configSwitches & 0x07)
	{
		case 0:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_3LIGHT_CA;
			signalHeadTypeA2 = signalHeadTypeB2 = signalAIsSingleHead()?SIGNAL_3LIGHT_CA:signalHeadTypeA1;
			break;
			
		case 1:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_3LIGHT_CC;
			signalHeadTypeA2 = signalHeadTypeB2 = signalAIsSingleHead()?SIGNAL_3LIGHT_CA:signalHeadTypeA1;
			break;

		case 2:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_3WIRE_RG_CA;
			signalHeadTypeA2 = signalHeadTypeB2 = signalAIsSingleHead()?SIGNAL_3LIGHT_CA:signalHeadTypeA1;
			break;		

		case 3:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_3WIRE_RG_CC;
			signalHeadTypeA2 = signalHeadTypeB2 = signalAIsSingleHead()?SIGNAL_3LIGHT_CA:signalHeadTypeA1;
			break;		

		case 4:
			signalHeadTypeA1 = signalHeadTypeB1 = SIGNAL_2WIRE_RG;
			signalHeadTypeA2 = signalHeadTypeB2 = signalAIsSingleHead()?SIGNAL_3LIGHT_CA:signalHeadTypeA1;
			break;		
		
		default:	
			signalHeadTypeA1 = signalHeadTypeA2 = signalHeadTypeB1 = signalHeadTypeB2 = SIGNAL_UNKNOWN;
			break;
	}

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

uint8_t auxDetectInputState = 0;

void debounceAuxInputs()
{
	static uint8_t clock_A=0, clock_B=0;
	uint8_t rawInput = ((~PING) & (_BV(PG3) | _BV(PG4)))>>3;
	uint8_t delta = rawInput ^ auxDetectInputState;
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;
	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.
	changes = ~((~delta) | clock_A | clock_B);
	auxDetectInputState ^= changes;	
}

/*
Basic algorithm:
 - Do detection, set detector outputs
 - Get codeline status
 - Translate codeline to aspect
 - Translate aspect to physical signals
*/


int main(void)
{
	

	// Deal with watchdog first thing
	MCUSR = 0;					// Clear reset status
	wdt_enable(WDTO_1S);

	// DDR = 0 - Input / 1 - Output

	// Initialize ports 
	// Pin Assignments for PORTA/DDRA
	//  PA0 - B Approach Diverging Input
	//  PA1 - B Advance Approach Input
	//  PA2 - B Local Block Input
	//  PA3 - A Approach Input
	//  PA4 - A Approach Diverging Input
	//  PA5 - A Advance Approach Input
	//  PA6 - A Local Block Input
	//  PA7 - DIP Switch 10
	DDRA  = 0b00000000;
	PORTA = 0b10000000;

	// Pin Assignments for PORTB/DDRB
	//  PB0 - B Approach Input
	//  PB1 - SCK (output)
	//  PB2 - MOSI (output)
	//  PB3 - MISO (in,pulled up)
	//  PB4 - B Occupancy Indicator (out)
	//  PB5 - A Occupancy Indicator (out)
	//  PB6 - Aux Indicator (out)
	//  PB7 - Optical Occupancy Indicator (out)
	DDRB  = 0b11110110;
	PORTB = 0b00001000;

	// Pin Assignments for PORTC/DDRC
	//  PC0 - DIP Switch 1 (in, pulled up)
	//  PC1 - DIP Switch 2 (in, pulled up)
	//  PC2 - DIP Switch 3 (in, pulled up)
	//  PC3 - DIP Switch 4 (in, pulled up)
	//  PC4 - DIP Switch 5 (in, pulled up)
	//  PC5 - DIP Switch 6 (in, pulled up)
	//  PC6 - DIP Switch 7 (in, pulled up)
	//  PC7 - DIP Switch 8 (in, pulled up)
	DDRC  = 0b00000000;
	PORTC = 0b11111111;

	
	// Pin Assignments for PORTD/DDRD
	//  PD0 - N/C
	//  PD1 - A Green 1  (out)
	//  PD2 - A Yellow 1 (out)
	//  PD3 - A Red 1    (out)
	//  PD4 - A Green 2  (out)
	//  PD5 - A Yellow 2 (out)
	//  PD6 - A Red 2    (out)
	//  PD7 - B Red 1    (out)
	DDRD  = 0b11111111;
	PORTD = 0b00000000;

	// Pin Assignments for PORTE/DDRE
	//  PE0 - RX (N/C)
	//  PE1 - TX (N/C)
	//  PE2 - N/C
	//  PE3 - B Green 1  (out)
	//  PE4 - SCL (out)
	//  PE5 - SDA (in)
	//  PE6 - B Yellow 1 (out)
	//  PE7 - B Green 2  (out)
	DDRE  = 0b11011111;
	PORTE = 0b00000000;
	
	// Pin Assignments for PORTE/DDRE
	//  PF0 - Analog 0 - Setpoint 1
	//  PF1 - Analog 1 - Setpoint 2
	//  PF2 - Analog 2 - Detector 1
	//  PF3 - Analog 3 - Detector 2
	//  PF4 - Analog 4 - A Red/Green balance
	//  PF5 - Analog 5 - B Red/Green balance
	//  PF6 - B Yellow 2 (out)
	//  PF7 - B Red 2    (out)
	DDRF  = 0b11000000;
	PORTF = 0b00000000;

	// Pin Assignments for PORTE/DDRE
	//  PG0 - A Occupancy Enable (out)
	//  PG1 - B Occupancy Enable (out)
	//  PG2 - DIP Swith 9 (in, pulled up)
	//  PG3 - A Occupancy Input (in)
	//  PG4 - B Occupancy Input (in)
	//  PG5 - N/C (/RESET line)
	//  PG6 - N/C (doesn't exist)
	//  PG7 - N/C (doesn't exist)
	DDRG  = 0b00000011;
	PORTG = 0b00011100;

	
	setOccupancyAOff();
	setOccupancyALEDOff();
	setOccupancyBOff();
	setOccupancyBLEDOff();

	setOccupancyIRLEDOff();
	setAuxLEDOff();

	initialize10HzTimer();
	initialize4000HzTimer();

	initializeDetectors();
	
	sei();

	while(1)
	{
		wdt_reset();

		// If all the analog inputs have been read, the flag will be set and we
		// can then process the analog detector inputs
		if (eventFlags & EVENT_DO_BD_READ)
		{
			setAuxLED(BLINK);
			
			// Do all the analog magic
			processDetectors();

			// Read the auxilliary occupancy inputs
			// Both are inverted, so flip the bit around so that auxDetectInputState is
			// positive logic 
			debounceAuxInputs();
			debounceConfig();

			if (detectorStatus & _BV(DETECT_DCC_A))
				setOccupancyALED(ON);
			else if (auxDetectInputState & _BV(DETECT_DCC_A))
				setOccupancyALED(BLINK);
			else
				setOccupancyALED(OFF);

			if (detectorStatus & _BV(DETECT_DCC_B))
				setOccupancyBLED(ON);
			else if (auxDetectInputState & _BV(DETECT_DCC_B))
				setOccupancyBLED(BLINK);
			else
				setOccupancyBLED(OFF);

			if (detectorStatus & _BV(DETECT_IR))
				setOccupancyIRLED(ON);
			else if (detectorStatus & _BV(ERROR_IR))
				setOccupancyIRLED(BLINK);
			else
				setOccupancyIRLED(OFF);

			// Set the output drivers according to what we're seeing on our own detectors
			if ((detectorStatus & (_BV(DETECT_DCC_A) | _BV(DETECT_IR))) || (auxDetectInputState & _BV(DETECT_DCC_A)))
				setOccupancyAOn();
			else
				setOccupancyAOff();

			if ((detectorStatus & (_BV(DETECT_DCC_B) | _BV(DETECT_IR))) || (auxDetectInputState & _BV(DETECT_DCC_B)))
				setOccupancyBOn();
			else
				setOccupancyBOff();

			// Clear the flag and start the next chain of conversions
			eventFlags &= ~(EVENT_DO_BD_READ);
		}

		if (EVENT_DO_ADC_RUN == (eventFlags & (EVENT_DO_ADC_RUN | EVENT_DO_BD_READ)))
		{
			// If the ISR tells us it's time to run the ADC again and we've handled the last read, 
			// start the ADC again
			ADCSRA |= _BV(ADSC);
			triggerIRDetector();
		}
		
		// Read what we see on the MSS bus
		processMSSCodeline();
		
		// Convert codeline indications to signal head aspects
		translateCodelineToIndications();
	}
}

void translateCodelineToIndications()
{
	uint8_t newSignalAspectA1 = ASPECT_OFF;
	uint8_t newSignalAspectA2 = ASPECT_OFF;	
	uint8_t newSignalAspectB1 = ASPECT_OFF;
	uint8_t newSignalAspectB2 = ASPECT_OFF;	

	if (signalAIsSingleHead())
	{
		switch(codeLineB)
		{
			case INDICATION_APPROACH_DIVERGING:
			case INDICATION_ADVANCE_APPROACH:
				newSignalAspectA1 = newSignalAspectA2 = ASPECT_FL_YELLOW;
				break;
			case INDICATION_APPROACH:
				newSignalAspectA1 = newSignalAspectA2 = ASPECT_YELLOW;
				break;
			case INDICATION_CLEAR:
				newSignalAspectA1 = newSignalAspectA2 = ASPECT_GREEN;
				break;
			case INDICATION_STOP:
			default:
				newSignalAspectA1 = newSignalAspectA2 = ASPECT_RED;
		}		
	}
	else
	{
		// Dual headed signals
		switch(codeLineB)
		{
			case INDICATION_APPROACH_DIVERGING:
				newSignalAspectA1 = ASPECT_YELLOW;
				newSignalAspectA2 = ASPECT_YELLOW;
				break;
				
			case INDICATION_ADVANCE_APPROACH:
				newSignalAspectA1 = ASPECT_FL_YELLOW;
				newSignalAspectA2 = ASPECT_RED;
				break;
				
			case INDICATION_APPROACH:
				newSignalAspectA1 = ASPECT_YELLOW;
				newSignalAspectA2 = ASPECT_RED;
				break;
				
			case INDICATION_CLEAR:
				newSignalAspectA1 = ASPECT_GREEN;
				newSignalAspectA2 = ASPECT_RED;
				break;

			case INDICATION_STOP:
			default:
				newSignalAspectA1 = newSignalAspectA2 = ASPECT_RED;
		}			
	}

	if (signalBIsSingleHead())
	{
		switch(codeLineA)
		{
			case INDICATION_APPROACH_DIVERGING:
			case INDICATION_ADVANCE_APPROACH:
				newSignalAspectB1 = newSignalAspectB2 = ASPECT_FL_YELLOW;
				break;
			case INDICATION_APPROACH:
				newSignalAspectB1 = newSignalAspectB2 = ASPECT_YELLOW;
				break;
			case INDICATION_CLEAR:
				newSignalAspectB1 = newSignalAspectB2 = ASPECT_GREEN;
				break;
			case INDICATION_STOP:
			default:
				newSignalAspectB1 = newSignalAspectB2 = ASPECT_RED;
		}		
	}
	else
	{
		// Dual headed signals
		switch(codeLineA)
		{
			case INDICATION_APPROACH_DIVERGING:
				newSignalAspectB1 = ASPECT_YELLOW;
				newSignalAspectB2 = ASPECT_YELLOW;
				break;
				
			case INDICATION_ADVANCE_APPROACH:
				newSignalAspectB1 = ASPECT_FL_YELLOW;
				newSignalAspectB2 = ASPECT_RED;
				break;
				
			case INDICATION_APPROACH:
				newSignalAspectB1 = ASPECT_YELLOW;
				newSignalAspectB2 = ASPECT_RED;
				break;
				
			case INDICATION_CLEAR:
				newSignalAspectB1 = ASPECT_GREEN;
				newSignalAspectB2 = ASPECT_RED;
				break;

			case INDICATION_STOP:
			default:
				newSignalAspectB1 = newSignalAspectB2 = ASPECT_RED;
		}			
	}

	if (signalsApproachLit())
	{
		if (codeLineA != INDICATION_STOP)
		{
			newSignalAspectA1 = ASPECT_OFF;
			if (signalAIsDualHead())
				newSignalAspectA2 = ASPECT_OFF;
		}

		if (codeLineB != INDICATION_STOP)
		{
			newSignalAspectB1 = ASPECT_OFF;
			if (signalAIsDualHead())
				newSignalAspectB2 = ASPECT_OFF;
		}
	}
	
	// Reload the signal type configuration
	signalConfigToSignalType();
	
	signalAspectA1 = newSignalAspectA1;
	signalAspectA2 = newSignalAspectA2;
	signalAspectB1 = newSignalAspectB1;
	signalAspectB2 = newSignalAspectB2;

}


