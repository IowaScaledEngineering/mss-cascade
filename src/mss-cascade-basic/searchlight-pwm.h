/*************************************************************************
Title:    MSS-CASCADE-BASIC Searhlight PWM Values
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
          Based on the work of David Johnson-Davies - www.technoblogy.com - 23rd October 2017
           and used under his Creative Commons Attribution 4.0 International license
File:     $Id: $
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

#ifndef _SEARCHLIGHT_PWM_H_
#define _SEARCHLIGHT_PWM_H_

/* The PWM values for the searchlight transitions are stored as a uint16 in program space
  11111100 00000000
  54321098 76543210
  xUUUUUDD DDDRRRRR
  
  The color going up (meaning we're transitioning to that aspect) is stored in U
  The color going down (meaning we're transitioning from that aspect) is stored in D
  The red flash between green and yellow is stored in R
*/

#define URD_TO_UINT16(u, r, d)   ((((u) & 0x1F)<<10) | (((d) & 0x1F)<<5) | ((r) & 0x1F))

const uint16_t const searchlightPWMsThroughRed[32] PROGMEM = 
{ 
	URD_TO_UINT16( 27,  0,  0),
	URD_TO_UINT16( 17,  0,  0),
	URD_TO_UINT16( 12,  0,  0),
	URD_TO_UINT16(  0,  0,  0),
	URD_TO_UINT16(  0, 17,  0),
	URD_TO_UINT16(  0, 25,  0),
	URD_TO_UINT16(  0, 25,  0),
	URD_TO_UINT16(  0, 17,  0),
	URD_TO_UINT16(  0,  0,  0),
	URD_TO_UINT16(  0,  0, 12),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  7, 17),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  0, 22),
	URD_TO_UINT16(  0,  0, 22),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  7, 17),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  0, 12),
	URD_TO_UINT16(  0, 12,  7),
	URD_TO_UINT16(  0, 17,  0),
	URD_TO_UINT16(  0, 22,  0),
	URD_TO_UINT16(  0, 17,  0),
	URD_TO_UINT16(  0, 12,  7),
	URD_TO_UINT16(  0,  0, 12),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  0, 22),
	URD_TO_UINT16(  0,  0, 27),
	URD_TO_UINT16(  0,  0, 31)
};

const uint16_t const searchlightPWMsInvolvingRed[] PROGMEM = { 
	URD_TO_UINT16( 31,  0,  0),
	URD_TO_UINT16( 27,  0,  0),
	URD_TO_UINT16( 22,  0,  0),
	URD_TO_UINT16( 17,  0,  0),
	URD_TO_UINT16( 12,  0,  0),
	URD_TO_UINT16(  5,  0,  0),
	URD_TO_UINT16(  0,  0,  0),
	URD_TO_UINT16(  0,  0,  5),
	URD_TO_UINT16(  0,  0, 12),
	URD_TO_UINT16(  0,  0, 17),
	URD_TO_UINT16(  0,  0, 22),
	URD_TO_UINT16(  0,  0, 27),
	URD_TO_UINT16(  0,  0, 31)
};

#endif

