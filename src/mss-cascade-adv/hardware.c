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
	i ^= (OPTION_A_APPROACH_LIGHTING | OPTION_B_RESERVED | OPTION_C_SEARCHLIGHT_MODE | OPTION_D_RESERVED | OPTION_E_CONFIG_MODE);

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
