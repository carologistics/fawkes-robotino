/***************************************************************************
 *  pinout.h - Pinout defines for ArduinoSketch
 * 
 *  Created: Tue Jan 06 03:32:57 2023
 *  Copyright 	2023  Tim Wendt
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

#ifndef __ARDUINO_PINOUT_HEADER_H_
#define __ARDUINO_PINOUT_HEADER_H_

#define MOTOR_XYZ_STEP_OUT PORTD
#define MOTOR_XYZ_DIR_OUT PORTD
#define MOTOR_A_STEP_OUT PORTB
#define MOTOR_A_DIR_OUT PORTB

#define MOTOR_X_STEP_SHIFT 2
#define MOTOR_Y_STEP_SHIFT 3
#define MOTOR_Z_STEP_SHIFT 4

#define MOTOR_X_DIR_SHIFT 5
#define MOTOR_Y_DIR_SHIFT 6
#define MOTOR_Z_DIR_SHIFT 7

#define MOTOR_A_STEP_SHIFT 4
#define MOTOR_A_DIR_SHIFT 5

/*
 * the following defines are for convenience only.
 * As different kinds of pins (e.g. step/dir/enable) are on the same port
 * every operation needs to make sure that only the used pins are changed.
 * For this purpose, the following masks can be logical anded with the previous state
 * leaving all other pins unchanged.
*/

#define MOTOR_XYZ_DIR_INV_MASK \
	~((1 << MOTOR_X_DIR_SHIFT) | (1 << MOTOR_Y_DIR_SHIFT) | (1 << MOTOR_Z_DIR_SHIFT))
#define MOTOR_A_DIR_INV_MASK ~(1 << MOTOR_A_DIR_SHIFT)

#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5
#define MOTOR_X_LIMIT_PIN 9

#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
#define MOTOR_Y_LIMIT_PIN 10

#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7
#define MOTOR_Z_LIMIT_PIN 11

#define MOTOR_A_ENABLE_PIN 8
#define MOTOR_A_STEP_PIN 12
#define MOTOR_A_DIR_PIN 13
#define MOTOR_A_OPEN_LIMIT_PIN A4

#define MOTOR_HOLD_PIN A4

#endif