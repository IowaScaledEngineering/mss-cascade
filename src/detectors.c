#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "mss-cascade.h"

volatile uint16_t adcValue[6];
uint8_t detectorOnDelayCount[2] = {0, 0};
uint8_t detectorOffDelayCount[2] = {0, 0};
uint8_t detectorOnDelay[2] = {4, 4};
uint8_t detectorOffDelay[2] = {25, 25};
uint8_t detectorStatus = 0;


void initializeADC()
{
	for(uint8_t i=0; i<sizeof(adcValue)/sizeof(adcValue[0]); i++)
		adcValue[i] = 0;

	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x40;  // AVCC reference, ADC0 starting channel
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 64 prescaler, ~8.3kconv / s
	ADCSRB = 0x00; // Free running mode
	DIDR0  = 0x3F;  // Turn ADC pins 0-5 into analog inputs
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}

void initializeTMD26711()
{

	// Initialize TMD26711 (bit 0x80 set to indicate command)
	writeByte(TMD26711_ADDR, 0x80|0x00, 0x00);   // Start with everything disabled
	writeByte(TMD26711_ADDR, 0x80|0x01, 0xFF);   // Minimum ATIME
	writeByte(TMD26711_ADDR, 0x80|0x02, 0xFF);   // Maximum integration time
	writeByte(TMD26711_ADDR, 0x80|0x03, 0xFF);   // Minimum wait time
	
	// Note: IRQ not currently used
	writeByte(TMD26711_ADDR, 0x80|0x08, 0x00);   // Set interrupt low threshold to 0x0000
	writeByte(TMD26711_ADDR, 0x80|0x09, 0x00);
	writeByte(TMD26711_ADDR, 0x80|0x0A, 0x00);   // Set interrupt low threshold to 0x0300
	writeByte(TMD26711_ADDR, 0x80|0x0B, 0x03);
	writeByte(TMD26711_ADDR, 0x80|0x0C, 0x10);   // Single out-of-range cycle triggers interrupt

	writeByte(TMD26711_ADDR, 0x80|0x0D, 0x00);   // Long wait disabled
	writeByte(TMD26711_ADDR, 0x80|0x0E, PPULSE_DEFAULT); // Pulse count
	writeByte(TMD26711_ADDR, 0x80|0x0F, 0x20);   // 100% LED drive strength, Use channel 1 diode (ch 1 seems less sensitive to fluorescent light)

	writeByte(TMD26711_ADDR, 0x80|0x00, 0x27);   // Power ON, Enable proximity, Enable proximity interrupt (not used currently)
}

void triggerIRDetector()
{
	if (detectorStatus & _BV(ERROR_IR))
		initializeTMD26711();

	writeByte(TMD26711_ADDR, 0x80|0x0E, PPULSE_DEFAULT);
}

void readIRDetector()
{
	static uint8_t count = 0;  // 256 decisecs * 60 (long delay mode) = 15360 max count
	uint16_t proximity;
	uint8_t ack;
	uint8_t on_debounce = ON_DEBOUNCE_DEFAULT;
	uint8_t off_debounce = OFF_DEBOUNCE_DEFAULT;

	ack = readWord(TMD26711_ADDR, 0x80|0x20|0x18, &proximity);  // Read data register (0x80 = command, 0x20 = auto-increment)

	if (!ack)
	{
		detectorStatus &= ~_BV(DETECT_IR);
		detectorStatus |= _BV(ERROR_IR);
		count = 0;
		return;
	}
	detectorStatus &= ~_BV(ERROR_IR);
	
	if(!(detectorStatus & _BV(DETECT_IR)))
	{
		if ((proximity >= PROXIMITY_THRESHOLD))
		{
			// ON debounce
			count++;
			if(count > on_debounce)
			{
				detectorStatus |= _BV(DETECT_IR);
				count = 0;
			}
		}
		else
		{
			count = 0;
		}
	}
	else
	{
		if (proximity < PROXIMITY_THRESHOLD)
		{
			// OFF debounce
			count++;
			if(count > off_debounce)
			{
				detectorStatus &= ~(_BV(DETECT_IR));
				count = 0;
			}
		} else {
			count = 0;
		}
	}

}



void initializeDetectors()
{
	uint8_t channel = 0;
	for (channel = 0; channel < 2 ; channel++)
	{
		detectorOnDelay[channel] = eeprom_read_byte((uint8_t*)(CHANNEL0_ON_DELAY + channel));
		if (0xFF == detectorOnDelay[channel] || 0x00 == detectorOnDelay[channel])
		{
			eeprom_write_byte((uint8_t*)(CHANNEL0_ON_DELAY + channel), 4);
			detectorOnDelay[channel] = eeprom_read_byte((uint8_t*)(CHANNEL0_ON_DELAY + channel));
		}	


		detectorOffDelay[channel] = eeprom_read_byte((uint8_t*)(CHANNEL0_OFF_DELAY + channel));
		if (0xFF == detectorOffDelay[channel] || 0x00 == detectorOffDelay[channel])
		{
			eeprom_write_byte((uint8_t*)(CHANNEL0_OFF_DELAY + channel), 25);
			detectorOffDelay[channel] = eeprom_read_byte((uint8_t*)(CHANNEL0_OFF_DELAY + channel));
		}	
	}
	
	initializeADC();
	initializeTMD26711();
}

ISR(ADC_vect)
{
	static uint8_t workingChannel = 0;
	static uint16_t accumulator = 0;
	static uint8_t count = 0;
	
	accumulator += ADC;
	if (++count >= 64)
	{
		adcValue[workingChannel] = accumulator / 64;
		accumulator = 0;
		count = 0;
		workingChannel++;

		if (6 == workingChannel)
		{
			workingChannel = 0;
			eventFlags |= EVENT_DO_BD_READ;
		}

		ADMUX = (ADMUX & 0xF0) | (workingChannel & 0x07);

	}

	if (0 == (eventFlags & EVENT_DO_BD_READ))
	{
		// Trigger the next conversion.  Not using auto-trigger so that we can safely change channels
		ADCSRA |= _BV(ADSC);
	}
}


void processDetectors(void)
{	
	for(uint8_t channel = 0; channel < 2; channel++)
	{
		uint16_t threshold = adcValue[ADC_CHANNEL_SETPOINT_0 + channel];
		uint8_t channelMask = (1<<channel);

		// Introduce a bit of hysteresis. 
		// If the channel is currently "on", lower the threshold by 5 counts
		// If the channel is currently "off", raise the threshold by 5
		if ((detectorStatus & channelMask) && (threshold >= 5))
		{
			threshold -= 5;
		} 
		else if ( (0 == (detectorStatus & channelMask)) && (threshold <= (1023-5)) )
		{
			threshold += 5;
		}
	
		if (adcValue[ADC_CHANNEL_DETECTOR_0 + channel] > threshold)
		{
			// Current for the channel has exceeded threshold
			
			if (0 == (detectorStatus & channelMask))
			{
				// Channel is currently in a non-detecting state
				// Wait for the turnon delay before actually indicating on
				if (detectorOnDelayCount[channel] < detectorOnDelay[channel])
					detectorOnDelayCount[channel]++;
				else
				{
					detectorStatus |= channelMask;
					detectorOffDelayCount[channel] = 0;
				}
			} else {
				// Channel is currently in a detecting state
				detectorOffDelayCount[channel] = 0;
			}
		} else {
			// Current for the channel is under the threshold value
			if (detectorStatus & channelMask)
			{
				// Channel is currently in a non-detecting state
				if (detectorOffDelayCount[channel] < detectorOffDelay[channel])
					detectorOffDelayCount[channel]++;
				else
				{
					detectorStatus &= ~(channelMask);
					detectorOnDelayCount[channel] = 0;
				}
			} else {
				// Channel is currently in a detecting state
				detectorOnDelayCount[channel] = 0;
			}
		}
	}
	
	
	readIRDetector();
}

uint8_t auxDetectInputState = 0;

void debounceAuxDetectInputs()
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


