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
#include <Servo.h>

//#define DEBUG_MODE

/*
 * The current time tick is extracted from the TCNT1 register.
 * As this is a two byte register, but the data bus is only one byte wide
 * correct read is not trivial.
 * However, the timer peripheral implements a shadow register for the high byte
 * which is written when the low byte is read.
 * It is thus important, that the low byte is read before the high byte, so the shadow register is properly loaded.
*/

// Create stepper motor instances
AccelStepper stepperX(AccelStepper::DRIVER, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

// Create servo motor instance
Servo servoA;

// Variables to track motor state
bool motorXEnabled = false;
bool motorYEnabled = false;
bool motorZEnabled = false;

int Xdir, Ydir, Zdir;

// Function declarations
void calibrate();
void calibrateAxis(AccelStepper &stepper, int limitPin, int &direction, int extra);
void moveStepperAbsolute(AccelStepper &stepper, int absoluteSteps, int limitPin, int &direction);
void enableMotor(AccelStepper &stepper);
void disableMotor(AccelStepper &stepper);
bool isLimitSwitchEngaged(int limitPin);
void checkConditionsAndRun();
void readMessage();
void gripperClose();
void gripperOpen();

#define DEFAULT_MAX_SPEED_X 4000
#define DEFAULT_MAX_ACCEL_X 7000

#define DEFAULT_MAX_SPEED_Y 4000
#define DEFAULT_MAX_ACCEL_Y 7000

#define DEFAULT_MAX_SPEED_Z 4000
#define DEFAULT_MAX_ACCEL_Z 7000

#define SECOND_CAL_MAX_SPEED 500
#define SECOND_CAL_MAX_ACC 1000

#define STATUS_MOVING 0
#define STATUS_IDLE 1
#define STATUS_ERROR 2

#define STATUS_OPEN 1 // DO WE NEED THIS ? NOT USED ANYWHERE HERE
#define STATUS_CLOSED 0 // DO WE NEED THIS ? NOT USED ANYWHERE HERE

char status_array_[] = {'M', 'I', 'E'};

bool open_gripper = false;

int gripper_open_angle;
int gripper_close_angle;

int cur_status = STATUS_IDLE;

int loop_nr = 0;

unsigned long prevMillisO, prevMillisC;

#define BUFFER_SIZE 256
char   buffer_[BUFFER_SIZE];
byte   buf_i_ = 0;
String errormessage;

void send_packet(int status_, int value_to_send) {
	Serial.print(AT);
	Serial.print(status_array_[status_]);
	Serial.print(value_to_send);
	Serial.print("\r\n");
}

inline int convert_to_check_sum(int i) {
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
void send_status() {
	byte checksum = 0;
	Serial.print(AT); //checksum = 181
	Serial.print(status_array_[cur_status]);
	checksum += (byte)status_array_[cur_status];
	Serial.print(" "); //checksum = 32
	if (cur_status == STATUS_ERROR) {
		Serial.print(errormessage);
	} else { // send all the information while moving and while idle
		cur_pos = stepperX.currentPosition();
		Serial.print(cur_pos);
		checksum += convert_to_check_sum(cur_pos);

		Serial.print(" "); //checksum = 32

		cur_pos = stepperY.currentPosition();
		Serial.print(cur_pos);
		checksum += convert_to_check_sum(cur_pos);

		Serial.print(" "); //checksum = 32

		cur_pos = -stepperZ.currentPosition();
		Serial.print(cur_pos);
		checksum += convert_to_check_sum(cur_pos);

		Serial.print(" "); //checksum = 32

		cur_pos = ((open_gripper))?gripper_open_angle:gripper_close_angle; // the angle value for the close and open as set for the gripper
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

int send_gripper_status() {
  //THE CHECKSUM 50 AND 180 need to be inverted if the gripper open / closed status seems inverted in fawkes. Check and update !!!
	if (open_gripper) {
		Serial.print("OPEN"); //checksum = 50
		return 50;
	}

	Serial.print("CLOSED"); //checksum = 186
	return 186;
}

void set_status(int status_) {
	if (cur_status != status_) {
		cur_status = status_;
		send_status();
	}
}

void double_calibrate() {
	// first fast calibration run
	calibrate();
	// reduce speed to a minimum
	float speeds[3] = {stepperX.speed(), stepperY.speed(), stepperZ.speed()};
	float accs[3]   = {stepperX.acceleration(), stepperY.acceleration(), stepperZ.acceleration()};
	set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC, stepperX);
	set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC, stepperY);
	set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC, stepperZ);
	// calibrate a second time
	calibrate();
	// after calibration the old speed and acc values are used
	set_new_speed_acc(speeds[0], accs[0], stepperX);
	set_new_speed_acc(speeds[1], accs[1], stepperY);
	set_new_speed_acc(speeds[2], accs[2], stepperZ);
}

void calibrateAxis(AccelStepper &stepper, int limitPin, int direction, int extra) {
    // Ensure motor is enabled
    enableMotor(stepper);

    // Move towards the end switch until triggered
    stepper.move(direction*40000);
    while (!isLimitSwitchEngaged(limitPin)) {
        stepper.run();
    }

    // Stop at the end switch and set position to zero
    stepper.setCurrentPosition(0);
    Serial.println("Axis calibrated.");

    stepper.move(-direction*(400+extra)); // move slightly in the opposite direction from limit switch after calibration
    while(stepper.distanceToGo()){
      stepper.run();
    }

    // Disable motor after calibration
    disableMotor(stepper);
}

void calibrate() {
    set_status(STATUS_MOVING);
    // Calibrate X axis
    calibrateAxis(stepperX, MOTOR_X_LIMIT_PIN, -1, 800);

    // Calibrate Y axis
    calibrateAxis(stepperY, MOTOR_Y_LIMIT_PIN, -1, 100);

    // Calibrate Z axis
    calibrateAxis(stepperZ, MOTOR_Z_LIMIT_PIN, 1, 200); // using inverted direction for Z-axis calibration

    set_status(STATUS_IDLE);
}

void reach_end_handle(AccelStepper &stepper, byte extra) {
	stepper.stop();
	stepper.setCurrentPosition(0L);
	stepper.move(-400 - extra);
}

void set_new_pos(long new_pos, AccelStepper &stepper) {
	stepper.enableOutputs();
	stepperY.enableOutputs();// IS IT NECESSARY TO ENABLE THIS??
	stepperZ.enableOutputs();// IS IT NECESSARY TO ENABLE THIS??
	stepper.moveTo(new_pos);
	set_status(
	  STATUS_MOVING); // status is always only changed on no interrupt code level, hence no race condition occurs here
}

void set_new_rel_pos(long new_rel_pos, AccelStepper &stepper) {
	stepper.enableOutputs();
	stepperY.enableOutputs();// IS IT NECESSARY TO ENABLE THIS??
	stepperZ.enableOutputs();// IS IT NECESSARY TO ENABLE THIS??
	stepper.move(new_rel_pos);
	set_status(STATUS_MOVING);
}

void set_new_speed(float new_speed) {
  stepperX.setMaxSpeed(new_speed);
  stepperY.setMaxSpeed(new_speed);
  stepperZ.setMaxSpeed(new_speed);
}

void set_new_acc(float new_acc) {
  stepperX.setAcceleration(new_acc);
  stepperY.setAcceleration(new_acc);
  stepperZ.setAcceleration(new_acc);
}

inline void set_new_speed_acc(float new_speed, float new_acc, AccelStepper &stepper) {
	stepper.setMaxSpeed(new_speed);
  stepper.setAcceleration(new_acc);
}

// stop all the motors using the most recent acceleration values
void slow_stop_all() {
	stepperX.stop();
	stepperY.stop();
	stepperZ.stop();
  disableMotor(stepperX);
  disableMotor(stepperY);
  disableMotor(stepperZ);
}

// stop all motors and stop taking further commands till told to - NEED TO ADD THIS FEATURE!!!
void fast_stop_all() {
  void slow_stop_all();
  disableMotor(stepperX);
  disableMotor(stepperY);
  disableMotor(stepperZ);
  motorXEnabled = false;
  motorYEnabled = false;
  motorZEnabled = false;
}

void read_package() {
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
		//float opening_speed = motor_A.maxSpeed(); //get current openening speed
		bool  assumed_gripper_state_local;
		// this is used to store the assumed gripper state locally, to reduce calls to the function get_assumed_gripper_state

		switch (cur_cmd) {
      //Since gripper has been changed from stepper to servo, most of the CMD for motor A are commented out as they are not necessary. NEED TO REMOVE THOSE CMDS LATER !!!
		case CMD_X_NEW_POS: enableMotor(stepperX); moveStepperAbsolute(stepperX, new_value, MOTOR_X_LIMIT_PIN, Xdir); break;
		case CMD_Y_NEW_POS: enableMotor(stepperY); moveStepperAbsolute(stepperY, new_value, MOTOR_Y_LIMIT_PIN, Ydir); break;
		case CMD_Z_NEW_POS: enableMotor(stepperZ); moveStepperAbsolute(stepperZ, -new_value, MOTOR_Z_LIMIT_PIN, Zdir); break;
		case CMD_STATUS_REQ: send_status(); break;
		case CMD_A_SET_TOGGLE_STEPS:
			//a_toggle_steps = new_value;
      //REMOVE CMD. NO LONGER NEED THIS
			send_status();
			break;
  /*#ifdef DEBUG_MODE
		case CMD_A_NEW_POS: //set_new_pos(new_value, motor_A);
      break;
  #endif*/
		case CMD_X_NEW_SPEED:
			set_new_speed_acc(new_value, 0.0, stepperX);
			send_status();
			break;
		case CMD_Y_NEW_SPEED:
			set_new_speed_acc(new_value, 0.0, stepperY);
			send_status();
			break;
		case CMD_Z_NEW_SPEED:
			set_new_speed_acc(new_value, 0.0, stepperZ);
			send_status();
			break;
		case CMD_A_NEW_SPEED:
			//set_new_speed_acc(new_value, 0.0, motor_A);
			send_status();
			break;
		case CMD_X_NEW_ACC:
			set_new_speed_acc(0.0, new_value, stepperX);
			send_status();
			break;
		case CMD_Y_NEW_ACC:
			set_new_speed_acc(0.0, new_value, stepperY);
			send_status();
			break;
		case CMD_Z_NEW_ACC:
			set_new_speed_acc(0.0, new_value, stepperZ);
			send_status();
			break;
		case CMD_A_NEW_ACC:
			//set_new_speed_acc(0.0, new_value, motor_A);
			send_status();
			break;
		case CMD_OPEN:
      prevMillisO = millis(); // using millis to show cur_status STATUS_MOVING for 1 second (called in loop() )
      cur_status = STATUS_MOVING;
			gripperOpen();
			break;
		case CMD_CLOSE:
      prevMillisC = millis(); // using millis to show cur_status STATUS_MOVING for 1 second (called in loop() )
      cur_status = STATUS_MOVING;
			gripperClose();
			break;
    case CMD_GRIP_CLOSE_ANG:
      gripper_close_angle = new_value;
      break;
    case CMD_GRIP_OPEN_ANG:
      gripper_open_angle = new_value;
      break;		
    case CMD_CALIBRATE: calibrate(); break;
		case CMD_DOUBLE_CALIBRATE: double_calibrate(); break;
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

bool isLimitSwitchEngaged(int limitPin) {
    return digitalRead(limitPin) == LOW;
}

void gripperClose() {
  servoA.write(gripper_close_angle);
}

void gripperOpen() {
  servoA.write(gripper_open_angle);
}

void moveStepperAbsolute(AccelStepper &stepper, int absoluteSteps, int limitPin, int &direction) {
    cur_status = STATUS_MOVING;
    enableMotor(stepper);
    int currentPosition = stepper.currentPosition();
    int targetPosition = absoluteSteps;

    if (targetPosition > currentPosition) {
        direction = 1;
    } else if (targetPosition < currentPosition) {
        direction = -1;
    }
    stepper.moveTo(targetPosition);
}

void checkConditionsAndRun() {
    bool anyMotorRunning = false;

    // Check and run each motor if needed
    if ((!isLimitSwitchEngaged(MOTOR_X_LIMIT_PIN) && stepperX.distanceToGo() != 0) || (isLimitSwitchEngaged(MOTOR_X_LIMIT_PIN) && Xdir == 1)) {
        anyMotorRunning = true;
        stepperX.run();
    }
    if ((!isLimitSwitchEngaged(MOTOR_Y_LIMIT_PIN) && stepperY.distanceToGo() != 0) || (isLimitSwitchEngaged(MOTOR_Y_LIMIT_PIN) && Ydir == 1)) {
        anyMotorRunning = true;
        stepperY.run();
    }
    if ((!isLimitSwitchEngaged(MOTOR_Z_LIMIT_PIN) && stepperZ.distanceToGo() != 0) || (isLimitSwitchEngaged(MOTOR_Z_LIMIT_PIN) && Zdir == -1)) {
        anyMotorRunning = true;
        stepperZ.run();
    }

    // Disable motors if no motor is running
    if (!anyMotorRunning) {
        disableMotor(stepperX);
        disableMotor(stepperY);
        disableMotor(stepperZ);
    }
}

void setup() {
	Serial.begin(115200);

	// initialize the LIMIT_PIN as an input per motor:
	pinMode(MOTOR_X_LIMIT_PIN, INPUT_PULLUP);
	pinMode(MOTOR_Y_LIMIT_PIN, INPUT_PULLUP);
	pinMode(MOTOR_Z_LIMIT_PIN, INPUT_PULLUP);

  pinMode(MOTOR_X_ENABLE_PIN, OUTPUT);

  // Attach the servo to pin 12
  servoA.attach(12);
  gripper_close_angle = 50;
  gripper_open_angle = 150;

	stepperX.disableOutputs(); // same pin for all of them
	/* stepperY.disableOutputs(); // same pin for all of them */
	/* stepperZ.disableOutputs(); // same pin for all of them */
	/* motor_A.disableOutputs(); // same pin for all of them */

	set_new_speed_acc(DEFAULT_MAX_SPEED_X, DEFAULT_MAX_ACCEL_X, stepperX);
	set_new_speed_acc(DEFAULT_MAX_SPEED_Y, DEFAULT_MAX_ACCEL_Y, stepperY);
	set_new_speed_acc(DEFAULT_MAX_SPEED_Z, DEFAULT_MAX_ACCEL_Z, stepperZ);

	//CHECK SUM of "AT HELLO +" = 628
	//lowByte(628) = 116
	Serial.println("AT HELLO +116");
	// while(!Serial.available()) {};

	send_status();

	set_status(STATUS_IDLE);

	stepperX.disableOutputs();

	//default behavior should be to calibrate and home on serial port open
	calibrate(); //SHOULD WE CALL DOUBLE CALIBRATE HERE INSTEAD? JUST TO BE ON A SAFER SIDE?
}

void loop() {

  if(millis() - prevMillisC > 1000 && prevMillisC != 0){
    cur_status = STATUS_IDLE;
    prevMillisC = 0;
    open_gripper = false;
  }

  if(millis() - prevMillisO > 1000 && prevMillisO != 0){
    cur_status = STATUS_IDLE;
    prevMillisO = 0;
    open_gripper = true;
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
    checkConditionsAndRun();

}
