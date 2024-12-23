/*************************************************************************
Title:    Hardware Configuration for MSS-SIDING
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
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

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#define LOOP_UPDATE_TIME_MS       50
#define TURNOUT_LOCKOUT_TIME_MS  200
#define STARTUP_LOCKOUT_TIME_MS  500

#define TCA9555_ADDR_000  0x20
#define TCA9555_ADDR_001  0x21
#define TCA9555_GPIN0        0
#define TCA9555_GPIN1        1
#define TCA9555_GPOUT0       2
#define TCA9555_GPOUT1       3
#define TCA9555_GPDDR0       6
#define TCA9555_GPDDR1       7

// TCA9555 0x20 - GPIO 0
//  IO07 - Input  - Option Switch E
//  IO06 - Input  - Option Switch D
//  IO05 - Input  - Option Switch C
//  IO04 - Input  - Option Switch B
//  IO03 - Input  - Option Switch A
//  IO02 - Input  - /SW3
//  IO01 - Input  - /SW4
//  IO00 - Input  - MSS v2


#define OPTION_MSS_V2                0x01
#define CONF_SW3_IN                  0x02
#define CONF_SW3_IN                  0x04

#define OPTION_COMMON_ANODE          0x02
#define OPTION_A_APPROACH_LIGHTING   0x08
#define OPTION_B_FOUR_ASPECT         0x10
#define OPTION_C_SEARCHLIGHT_MODE    0x20
#define OPTION_D_RESERVED            0x40
#define OPTION_E_RESERVED            0x80


// TCA9555 0x20 - GPIO 1
//  IO17 - Input  - B - AA In
//  IO16 - Input  - B - AD In
//  IO15 - Input  - A - S
//  IO14 - Input  - B - S
//  IO13 - Input  - A - A In
//  IO12 - Input  - A - AA In
//  IO11 - Input  - B - A In
//  IO10 - Input  - A - AD In

#define MSS_B_AA_IN           0x80
#define MSS_B_AD_IN           0x40
#define MSS_A_S               0x20
#define MSS_B_S               0x10
#define MSS_A_A_IN            0x08
#define MSS_A_AA_IN           0x04
#define MSS_B_A_IN            0x02
#define MSS_A_AD_IN           0x01

// TCA9555 0x21 - GPIO 0
//  IO07 - Output - Clear Conf Amber LED
//  IO06 - Output - AD Conf Amber LED
//  IO05 - Output - AD+AA Conf Amber LED
//  IO04 - Output - AA Conf Amber LED
//  IO03 - Output - A Conf Amber LED
//  IO02 - Output - S Conf Amber LED
//  IO01 - Output - Sig B Conf Blue LED
//  IO00 - Output - Sig A Conf Blue LED


#define CONF_LED_SIGNAL_A            0x01
#define CONF_LED_SIGNAL_B            0x02
#define CONF_LED_ASPECT_S            0x04
#define CONF_LED_ASPECT_A            0x08
#define CONF_LED_ASPECT_AA           0x10
#define CONF_LED_ASPECT_AD_AA        0x20
#define CONF_LED_ASPECT_AD           0x40
#define CONF_LED_ASPECT_CLR          0x80


// TCA9555 0x21 - GPIO 1
//  IO17 - Input  - Switch LH
//  IO16 - Input  - Switch UH
//  IO15 - Output - Conf Sig LH Red
//  IO14 - Output - Conf Sig LH Amber
//  IO13 - Output - Conf Sig LH Green
//  IO12 - Output - Conf Sig UH Red
//  IO11 - Output - Conf Sig UH Amber
//  IO10 - Output - Conf Sig UH Green

#define CONF_SWITCH_LOWER     0x80
#define CONF_SWITCH_UPPER     0x40
#define CONF_LED_LOWER_RED    0x20
#define CONF_LED_LOWER_YELLOW 0x10
#define CONF_LED_LOWER_GREEN  0x08
#define CONF_LED_UPPER_RED    0x04
#define CONF_LED_UPPER_YELLOW 0x02
#define CONF_LED_UPPER_GREEN  0x01



// Signal Port Connections
// These are in the order of:
//  Red address, bitmask
//  Yellow address, bitmask
//  Green address, bitmask

#define SIGNAL_HEAD_AU_DEF   &PORTA, _BV(PA7), &PORTA, _BV(PA6), &PORTA, _BV(PA5)
#define SIGNAL_HEAD_AL_DEF   &PORTA, _BV(PA4), &PORTA, _BV(PA3), &PORTB, _BV(PB0)
#define SIGNAL_HEAD_BU_DEF   &PORTB, _BV(PB6), &PORTB, _BV(PB5), &PORTB, _BV(PB4)
#define SIGNAL_HEAD_BL_DEF   &PORTB, _BV(PB3), &PORTB, _BV(PB2), &PORTB, _BV(PB1)


#endif
