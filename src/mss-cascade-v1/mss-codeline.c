#include <avr/io.h>
#include "mss-cascade.h"

#define DIR_A_STOP           ((PINA & _BV(PA6)) | (PORTG & _BV(PG0)))
#define DIR_A_APPROACH       (PINA & _BV(PA3))
#define DIR_A_DIV_APPROACH   (PINA & _BV(PA4))
#define DIR_A_ADV_APPROACH   (PINA & _BV(PA5))

#define DIR_B_STOP           ((PINA & _BV(PA2)) | (PORTG & _BV(PG1)))
#define DIR_B_APPROACH       (PINB & _BV(PB0))
#define DIR_B_DIV_APPROACH   (PINA & _BV(PA0))
#define DIR_B_ADV_APPROACH   (PINA & _BV(PA1))



CodelineStatus codeLineA = INDICATION_STOP;
CodelineStatus codeLineB = INDICATION_STOP;

void processMSSCodeline()
{
	if (DIR_A_STOP)
		codeLineA = INDICATION_STOP;
	else if (DIR_A_DIV_APPROACH)
		codeLineA = INDICATION_APPROACH_DIVERGING;
	else if (DIR_A_APPROACH)
		codeLineA = INDICATION_APPROACH;
	else if (DIR_A_ADV_APPROACH)
		codeLineA = INDICATION_ADVANCE_APPROACH;
	else
		codeLineA = INDICATION_CLEAR;

	if (DIR_B_STOP)
		codeLineB = INDICATION_STOP;
	else if (DIR_B_DIV_APPROACH)
		codeLineB = INDICATION_APPROACH_DIVERGING;
	else if (DIR_B_APPROACH)
		codeLineB = INDICATION_APPROACH;
	else if (DIR_B_ADV_APPROACH)
		codeLineB = INDICATION_ADVANCE_APPROACH;
	else
		codeLineB = INDICATION_CLEAR;
}


