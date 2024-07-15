#include <avr/io.h>
#include "mss-cascade.h"

inline void setOccupancyAOn()
{
	PORTG |= _BV(PG0);
}

inline void setOccupancyAOff()
{
	PORTG &= ~_BV(PG0);
}

inline void setOccupancyBOn()
{
	PORTG |= _BV(PG1);
}

inline void setOccupancyBOff()
{
	PORTG &= ~_BV(PG1);
}

inline void setOccupancyALEDOn()
{
	PORTB |= _BV(PB5);
}

inline void setOccupancyALEDOff()
{
	PORTB &= ~_BV(PB5);
}

void setOccupancyALED(uint8_t state)
{
	if (state)
		setOccupancyALEDOn();
	else
		setOccupancyALEDOff();	
}


inline void setOccupancyBLEDOn()
{
	PORTB |= _BV(PB4);
}

inline void setOccupancyBLEDOff()
{
	PORTB &= ~_BV(PB4);
}

void setOccupancyBLED(uint8_t state)
{
	if (state)
		setOccupancyBLEDOn();
	else
		setOccupancyBLEDOff();	
}

void setAuxLED(uint8_t state)
{
	if (state)
		setAuxLEDOn();
	else
		setAuxLEDOff();	
}

inline void setAuxLEDOn()
{
	PORTB |= _BV(PB6);
}

inline void setAuxLEDOff()
{
	PORTB &= ~_BV(PB6);
}

void setOccupancyIRLED(uint8_t state)
{
	if (state)
		setOccupancyIRLEDOn();
	else
		setOccupancyIRLEDOff();	
}

inline void setOccupancyIRLEDOn()
{
	PORTB |= _BV(PB7);
}

inline void setOccupancyIRLEDOff()
{
	PORTB &= ~_BV(PB7);
}


