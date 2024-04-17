/***************************************************************************
 *  commands.h - commands for ArduinoThread
 *
 *  Created: Mon Apr 04 15:40:32 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
 *                  2016  Nicolas Limpert
 *                  2023  Tim Wendt
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __ARDUINO_COMMANDS_HEADER_H_
#define __ARDUINO_COMMANDS_HEADER_H_

/**
 * @brief Mapping for all possible commands, that can be send to the arduino
 */
#define CMD_CALIBRATE 'C'
#define CMD_DOUBLE_CALIBRATE 'c'
#define CMD_X_NEW_POS 'X'
#define CMD_Y_NEW_POS 'Y'
#define CMD_Z_NEW_POS 'Z'
#define CMD_CLOSE 'G'
#define CMD_OPEN 'O'
#define CMD_STOP '.'
#define CMD_FAST_STOP ':'
#define CMD_STATUS_REQ 'S'
#define CMD_A_SET_TOGGLE_STEPS 'T'
#define CMD_X_NEW_SPEED 'x'
#define CMD_Y_NEW_SPEED 'y'
#define CMD_Z_NEW_SPEED 'z'
#define CMD_A_NEW_SPEED 'a'
#define CMD_SET_SPEED 'b'
#define CMD_X_NEW_ACC 'm'
#define CMD_Y_NEW_ACC 'n'
#define CMD_Z_NEW_ACC 'o'
#define CMD_A_NEW_ACC 'p'
#define CMD_SET_ACCEL 'q'
#ifdef DEBUG_MODE
#	define CMD_A_NEW_POS 'A'
#endif

#define AT "AT "
#define TERMINATOR '+'
class ArduinoHelper
{
public:
	static bool
	isValidSerialCommand(char cmd)
	{
		if (cmd == CMD_CALIBRATE || cmd == CMD_DOUBLE_CALIBRATE || cmd == CMD_X_NEW_POS
		    || cmd == CMD_Y_NEW_POS || cmd == CMD_Z_NEW_POS || cmd == CMD_CLOSE || cmd == CMD_OPEN
		    || cmd == CMD_STOP || cmd == CMD_FAST_STOP || cmd == CMD_STATUS_REQ
		    || cmd == CMD_A_SET_TOGGLE_STEPS || cmd == CMD_X_NEW_SPEED || cmd == CMD_Y_NEW_SPEED
		    || cmd == CMD_Z_NEW_SPEED || cmd == CMD_A_NEW_SPEED || cmd == CMD_SET_SPEED
		    || cmd == CMD_X_NEW_ACC || cmd == CMD_Y_NEW_ACC || cmd == CMD_Z_NEW_ACC
		    || cmd == CMD_A_NEW_ACC || cmd == CMD_SET_ACCEL
#ifdef DEBUG
		    || cmd == CMD_A_NEW_POS
#endif
		) {
			return true;
		}
		return false;
	}
};

#endif
