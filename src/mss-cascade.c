#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "mss-cascade.h"
#include "signalConfiguration.h"

// Global Variables
SignalHeadType signalHeadTypeA1 = SIGNAL_3LIGHT_CA;
SignalHeadType signalHeadTypeA2 = SIGNAL_3LIGHT_CA;
SignalHeadType signalHeadTypeB1 = SIGNAL_3LIGHT_CA;
SignalHeadType signalHeadTypeB2 = SIGNAL_3LIGHT_CA;
SignalAspect signalAspectA1 = ASPECT_OFF;
SignalAspect signalAspectA2 = ASPECT_OFF;
SignalAspect signalAspectB1 = ASPECT_OFF;
SignalAspect signalAspectB2 = ASPECT_OFF;

volatile uint8_t eventFlags = 0;
volatile uint8_t a_trim = 7;
volatile uint8_t b_trim = 7;


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

ISR(TIMER2_COMP_vect)
{
	static uint8_t slice = 0;

	if (++slice >= 32)
		slice = 0;

	IndicationToOutputs(setSignalA1, signalHeadTypeA1, signalAspectA1, (eventFlags & EVENT_1HZ_BLINK), slice, a_trim);
	IndicationToOutputs(setSignalA2, signalHeadTypeA2, signalAspectA2, (eventFlags & EVENT_1HZ_BLINK), slice, a_trim);
	IndicationToOutputs(setSignalB1, signalHeadTypeB1, signalAspectB1, (eventFlags & EVENT_1HZ_BLINK), slice, b_trim);
	IndicationToOutputs(setSignalB2, signalHeadTypeB2, signalAspectB2, (eventFlags & EVENT_1HZ_BLINK), slice, b_trim);

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
			debounceAuxDetectInputs();
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

	if (codeLineA > INDICATION_CLEAR)
		codeLineA = INDICATION_STOP;

	if (codeLineB > INDICATION_CLEAR)
		codeLineB = INDICATION_STOP;
	

	newSignalAspectA1 = clXlate[signalAAspectConfiguration()].upperHead[codeLineB];
	newSignalAspectA2 = clXlate[signalAAspectConfiguration()].lowerHead[codeLineB];

	newSignalAspectB1 = clXlate[signalBAspectConfiguration()].upperHead[codeLineA];
	newSignalAspectB2 = clXlate[signalBAspectConfiguration()].lowerHead[codeLineA];


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
			if (signalBIsDualHead())
				newSignalAspectB2 = ASPECT_OFF;
		}
	}
	
	// Reload the signal type configuration
	signalConfigToSignalType();
	
	a_trim = adcValue[5] / 32;
	b_trim = adcValue[4] / 32;
		
	signalAspectA1 = newSignalAspectA1;
	signalAspectA2 = newSignalAspectA2;
	signalAspectB1 = newSignalAspectB1;
	signalAspectB2 = newSignalAspectB2;

}


