/*************************************************************************
Title:    Modular Signal System Support Library
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     mss.c
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

#include "mss.h"


void mssPortInitialize(MSSPort_t* port)
{
	initDebounceState8(&port->debounce, 0x00);
	port->indication = INDICATION_STOP;
}

bool mssPortApproach(MSSPort_t* port)
{
	switch(port->indication)
	{
		case INDICATION_STOP:
		case INDICATION_APPROACH:
			return true;

		default:
			break;
	}
	return false;
}

MSSPortIndication_t mssPortWiresToIndication(bool S, bool A, bool AA, bool AD)
{
	MSSPortIndication_t indication = INDICATION_STOP;
	
	if (S)
		indication = INDICATION_STOP;
	else if (A && !AD)
		indication = INDICATION_APPROACH;
	else if (AD && AA)
		indication = INDICATION_APPROACH_DIVERGING_AA;
	else if (AD)
		indication = INDICATION_APPROACH_DIVERGING;
	else if (AA)
		indication = INDICATION_ADVANCE_APPROACH;
	else
		indication = INDICATION_CLEAR;
		
	return indication;
}


void mssReadPort(MSSPort_t* port, volatile uint8_t* adjacentPort, uint8_t adjacentMask,
	volatile uint8_t* approachPort, uint8_t approachMask,
	volatile uint8_t* advApproachPort, uint8_t advApproachMask,
	volatile uint8_t* divApproachPort, uint8_t divApproachMask)
{
	uint8_t mssInputState = 0;
	
	if ( *adjacentPort & adjacentMask )
		mssInputState |= MSS_MASK_ADJACENT;

	if ( *approachPort & approachMask )
		mssInputState |= MSS_MASK_APPROACH;

	if ( *advApproachPort & advApproachMask )
		mssInputState |= MSS_MASK_ADV_APPROACH;
		
	if ( divApproachPort && (*divApproachPort & divApproachMask) )
		mssInputState |= MSS_MASK_DIV_APPROACH;

	debounce8(mssInputState, &(port->debounce));

	mssInputState = getDebouncedState(&(port->debounce));
	
	port->indication = mssPortWiresToIndication(mssInputState & MSS_MASK_ADJACENT, 
			mssInputState & MSS_MASK_APPROACH, mssInputState & MSS_MASK_ADV_APPROACH, mssInputState & MSS_MASK_DIV_APPROACH);
}



void mssIndicationToSingleHeadAspect(MSSPortIndication_t indication, SignalAspect_t* aspect, uint8_t options, bool approachActive)
{
	// If we're approach lit and there's nothing on approach, turn off
	if (!approachActive && (options & MSS_ASPECT_OPTION_APPRCH_LIGHTING))
	{
		*aspect = ASPECT_OFF;
		return;
	}

	switch (indication)
	{
		case INDICATION_STOP:
		default:
			*aspect = ASPECT_RED;
			break;

		case INDICATION_APPROACH:
		case INDICATION_APPROACH_DIVERGING:
			*aspect = ASPECT_YELLOW;
			break;

		case INDICATION_ADVANCE_APPROACH:
			*aspect = (options & MSS_ASPECT_OPTION_FOUR_INDICATION)?ASPECT_FL_YELLOW:ASPECT_GREEN;
			break;

		case INDICATION_CLEAR:
			*aspect = ASPECT_GREEN;
			break;
	}
}



