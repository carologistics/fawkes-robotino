/***************************************************************************
 *  ArduinoSketch.ino - Ardunino sketch for controlling the gripper
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

#include "commands.h"
#include "pinout.h"

#include <AccelStepper.h>
#include <Wire.h>

//#define DEBUG_MODE

/*
 * The current time tick is extracted from the TCNT1 register.
 * As this is a two byte register, but the data bus is only one byte wide
 * correct read is not trivial.
 * However, the timer peripheral implements a shadow register for the high byte
 * which is written when the low byte is read.
 * It is thus important, that the low byte is read before the high byte, so the shadow register is properly loaded.
*/
#define CUR_TIME (TCNT1L | (unsigned int)((TCNT1H << 8)));

AccelStepper motor_X(MOTOR_X_STEP_SHIFT, MOTOR_X_DIR_SHIFT);
AccelStepper motor_Y(MOTOR_Y_STEP_SHIFT, MOTOR_Y_DIR_SHIFT);
AccelStepper motor_Z(MOTOR_Z_STEP_SHIFT, MOTOR_Z_DIR_SHIFT);

long a_toggle_steps = 240;

#define DEFAULT_MAX_SPEED_X 4000
#define DEFAULT_MAX_ACCEL_X 7000

#define DEFAULT_MAX_SPEED_Y 4000
#define DEFAULT_MAX_ACCEL_Y 7000

#define DEFAULT_MAX_SPEED_Z 4000
#define DEFAULT_MAX_ACCEL_Z 7000

#define DEFAULT_MAX_SPEED_A 7000
#define DEFAULT_MAX_ACCEL_A 15000

#define SECOND_CAL_MAX_SPEED 500
#define SECOND_CAL_MAX_ACC 1000

#define STATUS_MOVING 0
#define STATUS_IDLE 1
#define STATUS_ERROR 2

#define STATUS_OPEN 1
#define STATUS_CLOSED 0

#define servoPin 12 // HIGH to open gripper, LOW to close gripper

char status_array_[] = {'M', 'I', 'E'};

volatile bool movement_done_flag = false;

bool open_gripper = false;

int cur_status = STATUS_IDLE;

int loop_nr = 0;

//volatile unsigned long milliseconds = 0;
bool st = 1, entry = 1;
//unsigned long prevMillis = 0;
//const int refreshInterval = 20000; //Refresh interval in microseconds (20ms)

#define BUFFER_SIZE 256
char   buffer_[BUFFER_SIZE];
byte   buf_i_ = 0;
String errormessage;

void
enable_step_interrupt()
{
	TIMSK0 = 0x02; // enable interrupt of timer overflow
}

void
disable_step_interrupt()
{
	TIMSK0 = 0x0; // disable interrupt of timer overflow
}

void
send_packet(int status_, int value_to_send)
{
	Serial.print(AT);
	Serial.print(status_array_[status_]);
	Serial.print(value_to_send);
	Serial.print("\r\n");
}

inline int
convert_to_check_sum(int i)
{
	//using asci table to convert in to the number it would be casted to
	byte sum = 0;
	if (i == 0) {
		return 48; // if 0 then is it just the aci offset
	}

	if (i < 0) {
		sum += 45;  //add the offset for the minus sign
		i = i * -1; //remove the minus
	}

	int digitCount = 0;
	while (i != 0) {
		sum += i % 10;
		i /= 10;
		++digitCount;
	}

	sum += 48 * digitCount;

	return sum;
}

int cur_pos;
void
send_status()
{
	byte checksum = 0;
	Serial.print(AT); //checksum = 181
	Serial.print(status_array_[cur_status]);
	checksum += (byte)status_array_[cur_status];
	Serial.print(" "); //checksum = 32
	if (cur_status == STATUS_ERROR) {
		Serial.print(errormessage);
	} else { // send all the information while moving and while idle
		cur_pos = -motor_X.currentPosition();
		Serial.print(cur_pos);
		checksum += convert_to_check_sum(cur_pos);

		Serial.print(" "); //checksum = 32

		cur_pos = -motor_Y.currentPosition();
		Serial.print(cur_pos);
		checksum += convert_to_check_sum(cur_pos);

		Serial.print(" "); //checksum = 32

		cur_pos = -motor_Z.currentPosition();
		Serial.print(cur_pos);
		checksum += convert_to_check_sum(cur_pos);

		Serial.print(" "); //checksum = 32

		cur_pos = 0;
		Serial.print(cur_pos);
		checksum += convert_to_check_sum(cur_pos);

		Serial.print(" "); //checksum = 32

		checksum += send_gripper_status();
	}
	Serial.print("+"); //checksum = 43
	//SUM of all checksum =384 - 256 = 128;
	checksum += 128;
	Serial.print(checksum);
	Serial.print("\r\n");
}

int
send_gripper_status()
{
	if (open_gripper) {
		Serial.print("OPEN"); //checksum = 50
		return 50;
	}

	Serial.print("CLOSED"); //checksum = 186
	return 186;
}

void
set_status(int status_)
{
	if (cur_status != status_) {
		cur_status = status_;
		send_status();
	}
}

void
double_calibrate()
{
	// first fast calibration run
	calibrate();
	while (!movement_done_flag)
		;
	movement_done_flag = false;
	// reduce speed to a minimum
	float speeds[3] = {motor_X.get_speed(), motor_Y.get_speed(), motor_Z.get_speed()};
	float accs[3]   = {motor_X.get_acc(), motor_Y.get_acc(), motor_Z.get_acc()};
	set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC, motor_X);
	set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC, motor_Y);
	set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC, motor_Z);
	// calibrate a second time
	calibrate();
	while (!movement_done_flag)
		;
	movement_done_flag = false;
	// after calibration the old speed and acc values are used
	set_new_speed_acc(speeds[0], accs[0], motor_X);
	set_new_speed_acc(speeds[1], accs[1], motor_Y);
	set_new_speed_acc(speeds[2], accs[2], motor_Z);
}

void
calibrate()
{
	// while the x axis is not done calibrating, the gripper is not moving in y
	// and z direction, else this could be dangerous if a workpiece is placed
	// near the gripper
	bool x_done = false, y_done = false, z_done = false;
	do { //repeat calibration as long as not successfull
		motor_X.enableOutputs();
		motor_Y.enableOutputs();
		motor_Z.enableOutputs();
		noInterrupts();
		if (!x_done)
			motor_X.move(20000L);
		movement_done_flag = false;
		interrupts();
		// due to high step count, reaching end stops is guaranteed!
		set_status(
		  STATUS_MOVING); // status is always only changed on no interrupt code level, hence no race condition occurs here
		// This while loop controls permanently the state of the respective end stops and handles crashing into them
		// When all end stops are triggered simulatenously, additional latency is introduced.
		// The latency is maily due to the planning of the back movement.
		// One work around is to calibrate twice, the first time fast and the second time slowly.
		// (again inspired from grbl)
		// Additionally, the backwards movement should use different numbers of steps
		// to ensure that the end stops will not be triggerend simulatenously at the second calibration.
		while (!movement_done_flag && (!x_done)) {
			if (!x_done && digitalRead(MOTOR_X_LIMIT_PIN) == LOW) {
				x_done = true;
				reach_end_handle(motor_X, 0);
			}
		}
		movement_done_flag = false;
	} while (!x_done);
	do { //repeat calibration as long as not successfull
		noInterrupts();
		if (!y_done)
			motor_Y.move(20000L);
		if (!z_done)
			motor_Z.move(20000L);
		movement_done_flag = false;
		interrupts();
		// due to high step count, reaching end stops is guaranteed!
		set_status(
		  STATUS_MOVING); // status is always only changed on no interrupt code level, hence no race condition occurs here
		// This while loop controls permanently the state of the respective end stops and handles crashing into them
		// When all end stops are triggered simulatenously, additional latency is introduced.
		// The latency is maily due to the planning of the back movement.
		// One work around is to calibrate twice, the first time fast and the second time slowly.
		// (again inspired from grbl)
		// Additionally, the backwards movement should use different numbers of steps
		// to ensure that the end stops will not be triggerend simulatenously at the second calibration.
		while (!movement_done_flag && (!y_done || !z_done)) {
			if (!y_done && digitalRead(MOTOR_Y_LIMIT_PIN) == LOW) {
				y_done = true;
				reach_end_handle(motor_Y, 100);
			}
			if (!z_done && digitalRead(MOTOR_Z_LIMIT_PIN) == LOW) {
				z_done = true;
				reach_end_handle(motor_Z, 200);
			}
		}
		movement_done_flag = false;
	} while (!x_done || !y_done || !z_done);
}

void
Xcalibrate()
{
	bool x_done = false;
	do {
		motor_X.enableOutputs();
		noInterrupts();
		if (!x_done)
			motor_X.move(20000L);
		movement_done_flag = false;
		interrupts();
		// due to high step count, reaching end stops is guaranteed!
		set_status(STATUS_MOVING);
		while (!movement_done_flag && (!x_done)) {
			if (!x_done && digitalRead(MOTOR_X_LIMIT_PIN) == LOW) {
				x_done = true;
				reach_end_handle(motor_X, 0);
			}
		}
		movement_done_flag = false;
	} while (!x_done);
}

void
reach_end_handle(AccelStepper &motor, byte extra)
{
	motor.hard_stop();
	motor.setCurrentPosition(0L);
	motor.move(-400 - extra);
}

void
set_new_pos(long new_pos, AccelStepper &motor)
{
	motor.enableOutputs();
	motor_Y.enableOutputs();
	motor_Z.enableOutputs();
	noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
	motor.moveTo(new_pos);
	interrupts(); // activate interrupts again
	set_status(
	  STATUS_MOVING); // status is always only changed on no interrupt code level, hence no race condition occurs here
}

void
set_new_rel_pos(long new_rel_pos, AccelStepper &motor)
{
	motor.enableOutputs();
	motor_Y.enableOutputs();
	motor_Z.enableOutputs();
	noInterrupts();
	motor.move(new_rel_pos);
	interrupts();
	set_status(STATUS_MOVING);
}

void
set_new_speed(float new_speed)
{
	noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
	set_new_speed_acc(new_speed, -1, motor_X);
	set_new_speed_acc(new_speed, -1, motor_Y);
	set_new_speed_acc(new_speed, -1, motor_Z);
	interrupts(); // activate interrupts again
}

void
set_new_acc(float new_acc)
{
	noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
	set_new_speed_acc(-1, new_acc, motor_X);
	set_new_speed_acc(-1, new_acc, motor_Y);
	set_new_speed_acc(-1, new_acc, motor_Z);
	interrupts(); // activate interrupts again
}

inline void
set_new_speed_acc(float new_speed, float new_acc, AccelStepper &motor)
{
	motor.setMaxSpeedAcc(new_speed, new_acc);
}

// stop all the motors using the most recent acceleration values
void
slow_stop_all()
{
	motor_X.stop();
	motor_Y.stop();
	motor_Z.stop();
}

// stop all motors with infinite acceleration. Do not use if step counter should still be correct afterwards.
void
fast_stop_all()
{
	motor_X.hard_stop();
	motor_Y.hard_stop();
	motor_Z.hard_stop();
}

void
read_package()
{
	char next_char;
	while (true) {
		next_char = Serial.read();
		if (next_char == TERMINATOR) {
			// if we find the terminator character we can analyze the package now
			buffer_[buf_i_] = 0;
			// Set null character to get no trouble with sscanf // buf_i_ points now onto 0
			break;
		} else if (next_char == -1) {
			// if no serial data is available anymore, but package terminator not found yet:
			return; // cannot do anything now
		}
		buffer_[buf_i_++] = next_char; // other characters are added to the buffer
		if (buf_i_ >= BUFFER_SIZE) {
			// Buffer overflow. Strategy: flush buffer and start new. (This should not happen normally)
			buf_i_ = 0;
			return;
		}
	}

	// this point is only reached when a Terminator symbol was reached
	if (buf_i_ < 4) {
		buf_i_ = 0;
		return;
	} // skip too small packages // buffer flush

	byte package_start   = 0;
	bool package_located = false;

	for (package_start = 0; package_start < buf_i_ - 2; package_start++) {
		if (buffer_[package_start] == 'A' && buffer_[package_start + 1] == 'T'
		    && buffer_[package_start + 2] == ' ') {
			package_located = true;
			break; // explicitly break, otherwise package_start++ is executed once again
		}
	}

	if (!package_located) { // terminator symbol was reached but no package was located
		buf_i_ = 0;           // flush buffer
		return;
	}

	// this point is only reached when package was successfully located
	byte cur_i_cmd = package_start + 3;

	while (cur_i_cmd < buf_i_) {
		char cur_cmd   = buffer_[cur_i_cmd];
		long new_value = 0;
		if (ArduinoHelper::isValidSerialCommand(cur_cmd)) {
			if (sscanf(buffer_ + (cur_i_cmd + 1), "%ld", &new_value) <= 0) {
				buf_i_ = 0;
				return;
			} // flush and return if parsing error
		}
		bool assumed_gripper_state_local;
		// this is used to store the assumed gripper state locally, to reduce calls to the function get_assumed_gripper_state
		switch (cur_cmd) {
		case CMD_X_NEW_POS: set_new_pos(-new_value, motor_X); break;
		case CMD_Y_NEW_POS: set_new_pos(-new_value, motor_Y); break;
		case CMD_Z_NEW_POS: set_new_pos(-new_value, motor_Z); break;
		case CMD_STATUS_REQ: send_status(); break;
		case CMD_A_SET_TOGGLE_STEPS:
			a_toggle_steps = new_value;
			send_status();
			break;
		case CMD_X_NEW_SPEED:
			set_new_speed_acc(new_value, 0.0, motor_X);
			send_status();
			break;
		case CMD_Y_NEW_SPEED:
			set_new_speed_acc(new_value, 0.0, motor_Y);
			send_status();
			break;
		case CMD_Z_NEW_SPEED:
			set_new_speed_acc(new_value, 0.0, motor_Z);
			send_status();
			break;
		case CMD_A_NEW_SPEED: send_status(); break;
		case CMD_X_NEW_ACC:
			set_new_speed_acc(0.0, new_value, motor_X);
			send_status();
			break;
		case CMD_Y_NEW_ACC:
			set_new_speed_acc(0.0, new_value, motor_Y);
			send_status();
			break;
		case CMD_Z_NEW_ACC:
			set_new_speed_acc(0.0, new_value, motor_Z);
			send_status();
			break;
		case CMD_A_NEW_ACC: send_status(); break;
		case CMD_OPEN: open_gripper = HIGH; break;
		case CMD_CLOSE: open_gripper = LOW; break;
		case CMD_CALIBRATE: calibrate(); break;
		case CMD_DOUBLE_CALIBRATE: double_calibrate(); break;
		case CMD_X_CALIBRATE: Xcalibrate(); break;
		case CMD_SET_SPEED:
			set_new_speed(new_value);
			send_status();
			break;
		case CMD_SET_ACCEL:
			set_new_acc(new_value);
			send_status();
			break;
		case CMD_STOP:
			slow_stop_all();
			set_status(STATUS_IDLE);
			movement_done_flag = true;
			send_status();
			break;
		case CMD_FAST_STOP: fast_stop_all(); break;
		default:
#ifdef DEBUG_MODE
			send_packet(STATUS_ERROR, 15);
#endif
			break;
		}

		// move to next command
		while (cur_i_cmd < buf_i_) {
			++cur_i_cmd;
			if (buffer_[cur_i_cmd] == ' ') {
				++cur_i_cmd;
				break;
			}
		}
	}

	// sucked everything out of this package, flush it
	buf_i_ = 0;
}

void
setup()
{
	Serial.begin(115200);
	// Serial.setTimeout(0);

	TIMSK0 = 0; // turn off interrupts on Timer 0. micros() will thus NOT work anymore!
	/* Why use timer 0 for the step interrupt?
  *  Why use timer 2 for the pule interrupts?
  *  By this procedure it's made sure that the pulse interrupts fire BEFORE a new step interrupt is called, in case both are pending
  */

	TCCR1A = 0x0;
	TCCR1B = 0x3; // use 64 prescaler -> 4 us per cnt
	TIMSK1 = 0x0; // deactivate all interrupts with this timer

	// initialize the LIMIT_PIN as an input per motor:
	pinMode(MOTOR_X_LIMIT_PIN, INPUT_PULLUP);
	pinMode(MOTOR_Y_LIMIT_PIN, INPUT_PULLUP);
	pinMode(MOTOR_Z_LIMIT_PIN, INPUT_PULLUP);
	pinMode(MOTOR_A_OPEN_LIMIT_PIN, INPUT_PULLUP);

	pinMode(MOTOR_X_STEP_PIN, OUTPUT);
	pinMode(MOTOR_Y_STEP_PIN, OUTPUT);
	pinMode(MOTOR_Z_STEP_PIN, OUTPUT);

	pinMode(MOTOR_X_DIR_PIN, OUTPUT);
	pinMode(MOTOR_Y_DIR_PIN, OUTPUT);
	pinMode(MOTOR_Z_DIR_PIN, OUTPUT);

	pinMode(servoPin, OUTPUT);

	motor_X.setEnablePin(MOTOR_X_ENABLE_PIN, true);
	motor_Y.setEnablePin(MOTOR_Y_ENABLE_PIN, true);
	motor_Z.setEnablePin(MOTOR_Z_ENABLE_PIN, true);
	motor_X.disableOutputs(); // same pin for all of them
	/* motor_Y.disableOutputs(); // same pin for all of them */
	/* motor_Z.disableOutputs(); // same pin for all of them */
	/* motor_A.disableOutputs(); // same pin for all of them */

	set_new_speed_acc(DEFAULT_MAX_SPEED_X, DEFAULT_MAX_ACCEL_X, motor_X);
	set_new_speed_acc(DEFAULT_MAX_SPEED_Y, DEFAULT_MAX_ACCEL_Y, motor_Y);
	set_new_speed_acc(DEFAULT_MAX_SPEED_Z, DEFAULT_MAX_ACCEL_Z, motor_Z);

	//CHECK SUM of "AT HELLO +" = 628
	//lowByte(628) = 116
	Serial.println("AT HELLO +116");
	// while(!Serial.available()) {};

	send_status();

	set_status(STATUS_IDLE);

	motor_X.disableOutputs();

	// configure the pulse interrupt

	TCCR2A = 0x1; // just normal mode
	TCCR2B = 0x0; //no clock source, activate with TCCR2B=0x1; just direct io clock
	TCNT2  = 0;   // reset counter
	OCR2A  = 3;   //15; // start pulse, should be at least 650ns after setting direction
	OCR2B  = 254; // end pulse, should be at least 1.9us after starting pulse
	TIFR2  = 0x7; // clear already set flags
	TIMSK2 = 0x6; // activate both compare interrupts

	// configure the step interrupt

	TCCR0A = 0x2; // CTC mode
	TCCR0B = 0x2; // 0.5us per cnt, prescaler is 8
	OCR0A  = 70;  // 35us per step
	TCNT0  = 0;
	TIMSK0 = 0;   // start new
	TIFR0  = 0x7; // clear all interrupt flags

	enable_step_interrupt();

	interrupts();
	//default behavior should be to calibrate and home on serial port open
	calibrate();
}

int loop_number = 0;
void
loop()
{
	if (open_gripper) {
		digitalWrite(servoPin, HIGH);
	} else {
		digitalWrite(servoPin, LOW);
	}
	if (movement_done_flag) {
		motor_X.disableOutputs(); // same pin for all of them
		movement_done_flag = false;
		set_status(STATUS_IDLE);
	}

	read_package();

	if (cur_status != STATUS_MOVING) {
		loop_nr = 0;
		return;
	}

	if (loop_nr > 3000) {
		send_status();
		loop_nr = 0;
	} else {
		loop_nr++;
	}
}

volatile byte step_bits_xyz = 0;
volatile byte step_bits_a   = 0;
volatile bool pulse_done    = true;

ISR(TIMER0_COMPA_vect) // this is called every overflow of the timer 0
{
	static volatile bool occupied = false;
	if (occupied)
		return; // this interrupt is already called
	occupied = true;
	//milliseconds++;
	interrupts(); // we need interrupts here to catch all the incoming serial data and finish the pulse
	if (cur_status == STATUS_MOVING) {
		byte dir = 0, step = 0;
		bool movement_done;

		unsigned int time = CUR_TIME;
		movement_done     = motor_X.get_step(step, dir, time);
		movement_done &= motor_Y.get_step(step, dir, time);
		movement_done &= motor_Z.get_step(step, dir, time);
		movement_done_flag = movement_done;
		if (step) { // only step if really necessary
			while (!pulse_done)
				; // wait until the previous pulse is done // TODO:: remove after ensuring pulse is surely done here EVERY TIME!
			// Serial.println(dir,BIN);
			dir ^= 1 << MOTOR_X_DIR_SHIFT;
			dir ^= 1 << MOTOR_Y_DIR_SHIFT;
			MOTOR_XYZ_DIR_OUT =
			  (MOTOR_XYZ_DIR_OUT & MOTOR_XYZ_DIR_INV_MASK) | dir; // set the direction pins right
			step_bits_xyz = step;

			pulse_done = false;
			TCCR2B     = 0x1; // activate pulse interrupts // GO GO GO
		}
	}
	occupied = false;
}

ISR(TIMER2_COMPA_vect) // start of pulse
{
	MOTOR_XYZ_STEP_OUT |= step_bits_xyz;
}

ISR(TIMER2_COMPB_vect) // end of pulse
{
	MOTOR_XYZ_STEP_OUT &=
	  ~step_bits_xyz; // stopping the pulse has higher priority than turning off the timer
	TCCR2B     = 0x0; // stop timer
	TCNT2      = 0;   // reset cnt value
	pulse_done = true;
}
