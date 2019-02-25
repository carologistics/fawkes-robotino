#include <Wire.h>
#include <AccelStepper.h>

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


AccelStepper motor_X(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper motor_Y(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper motor_Z(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);
AccelStepper motor_A(1, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);

#define AT "AT "
#define TERMINATOR '+'

#define CMD_CALIBRATE 'C'

#define CMD_X_NEW_POS 'X'
#define CMD_Y_NEW_POS 'Y'
#define CMD_Z_NEW_POS 'Z'
#define CMD_OPEN 'O'
#define CMD_CLOSE 'G'
#define CMD_STATUS_REQ 'S'
#define CMD_SET_ACCEL '7'
#define CMD_SET_SPEED '9'

#define DEFAULT_MAX_SPEED_X 1500
#define DEFAULT_MAX_ACCEL_X 2000

#define DEFAULT_MAX_SPEED_Y 2000
#define DEFAULT_MAX_ACCEL_Y 3500

#define DEFAULT_MAX_SPEED_Z 2000
#define DEFAULT_MAX_ACCEL_Z 3500

#define DEFAULT_MAX_SPEED_A 4000
#define DEFAULT_MAX_ACCEL_A 5000


#define STATUS_MOVING 0
#define STATUS_IDLE 1
#define STATUS_ERROR 2

bool movement_done_flag = false;

bool open_gripper = false;
bool closed_gripper = false;

char status_array_[] = {'M', 'I', 'E'};

int steps_pending_X = 0;
int steps_pending_Y = 0;
int steps_pending_Z = 0;
int steps_pending_A = 0;

int cur_cmd = 0;

int cur_status = STATUS_IDLE;

#define BUFFER_SIZE 128
char buffer_[BUFFER_SIZE];
int buf_i_ = 0;
String errormessage;

int button_x_state = 0; // limit_x status
int button_y_state = 0; // limit_y status
int button_z_state = 0; // limit_z status

void send_packet(int status_, int value_to_send) {
    Serial.print(AT);
    Serial.print(status_array_[status_]);
    Serial.print(value_to_send);
    Serial.print("\r\n");
}

void send_moving() {
  Serial.print(AT);
  Serial.print("M");
  Serial.print("\r\n");
}

void send_idle() {
  Serial.print(AT);
  Serial.print("I ");
  Serial.print(-motor_X.currentPosition());
  Serial.print(" ");
  Serial.print(-motor_Y.currentPosition());
  Serial.print(" ");
  Serial.print(-motor_Z.currentPosition());
  Serial.print(" ");
  Serial.print(motor_A.currentPosition());
  Serial.print(" ");
  int open_button = digitalRead(MOTOR_A_OPEN_LIMIT_PIN);
  if(open_button == LOW){
        Serial.print("OPEN");
  }
  if(open_button == HIGH){
        Serial.print("CLOSED");
  }

  Serial.print("\r\n");
}

void send_error() {
  Serial.print(AT);
  Serial.print("E ");
  Serial.print(errormessage);
  Serial.print("\r\n");
}

void set_status(int status_) {
  if (cur_status != status_) {
    cur_status = status_;
    if (cur_status == STATUS_IDLE) {
      send_idle();
    } else if (cur_status == STATUS_MOVING) {
      send_moving();
    } else if (cur_status == STATUS_ERROR) {
      send_error();
    }
  }
}

// move motor to according end stop - dir is direction, (1 or -1)
void move_to_end_stop(int limit_pin, AccelStepper &motor, int dir) {
//  float motor_speed = motor.maxSpeed();
//  motor.setMaxSpeed( motor.maxSpeed() / 2.0);
  int button_state = digitalRead(limit_pin);

  while (button_state == HIGH) {
    button_state = digitalRead(limit_pin);
    if (button_state == LOW) {
      motor.setCurrentPosition(0L);
    } else {
      motor.move(1000 * dir);
      motor.run();
    }
  }

  // move out of the end stop
  // TODO: configure
  motor.setCurrentPosition(0L);
  motor.move(250 * dir * -1);
  while (motor.distanceToGo() != 0) {
 //   motor.setSpeed(DEFAULT_MAX_SPEED);
    motor.run();
  }

//  motor.setMaxSpeed(motor_speed);
}

bool calibrate_axis(int limit_pin, AccelStepper &motor, int dir) {
  int button_state = digitalRead(limit_pin);

  /*
  if (button_state == LOW) {
    //ERROR: We can't calibrate an axis when the end stop is already pressed!
    return false;
  }
  */
  motor.enableOutputs();

  move_to_end_stop(limit_pin, motor, dir);

  // we found the starting point - move to the other end of the axis to get
  // the number of steps to the other end.

  //move_to_end_stop(limit_pin, motor, 1);
  motor.disableOutputs();
  return true;
}


// maybe change this to home all 3 axis simultaneously
void calibrate() {
  set_status(STATUS_MOVING);

  if (!calibrate_axis(MOTOR_X_LIMIT_PIN, motor_X, 1)) {
    errormessage = "Can't calibrate axis X - end stop pressed!";
    set_status(STATUS_ERROR);
    return;
  }
  if (!calibrate_axis(MOTOR_Y_LIMIT_PIN, motor_Y, 1)) {
    errormessage = "Can't calibrate axis Y - end stop pressed!";
    set_status(STATUS_ERROR);
    return;
  }
  if (!calibrate_axis(MOTOR_Z_LIMIT_PIN, motor_Z, -1)) {
    errormessage = "Can't calibrate axis Z - end stop pressed!";
    set_status(STATUS_ERROR);
    return;
  }

//move_to_home(MOTOR_X_LIMIT_PIN, motor_A); // maybe enable home position via X limit Pin?

  set_status(STATUS_IDLE);
}

//void set_motor_move(long steps, int &steps_pending_identifier, AccelStepper &motor) {
void set_new_pos(long new_pos, AccelStepper &motor) {
  motor.enableOutputs();
//  steps_pending_identifier = abs(steps);
  noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
  motor.moveTo(new_pos);
  interrupts(); // activate interrupts again
  set_status(STATUS_MOVING); // status is always only changed on no interrupt code level, hence no race condition occurs here
}

void set_new_speed(long new_speed) {
  noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
  motor_X.setMaxSpeed(new_speed);
  motor_Y.setMaxSpeed(new_speed);
  motor_Z.setMaxSpeed(new_speed);
  motor_A.setMaxSpeed(new_speed);
  interrupts(); // activate interrupts again
}

void set_new_acc(long new_acc) {
  noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
  motor_X.setAcceleration(new_acc);
  motor_Y.setAcceleration(new_acc);
  motor_Z.setAcceleration(new_acc);
  motor_A.setAcceleration(new_acc);
  interrupts(); // activate interrupts again
}

void read_package() {
  int next_char;
  while(true) 
  {
    next_char = Serial.read();
    if(next_char == TERMINATOR){ // if we find the terminator character we can analyze the package now
      buffer_[buf_i_] = 0; // Set null character to get no trouble with sscanf //buf_i_ points now onto 0
      break;
    } else if(next_char == -1) { //if no serial data is available anymore, but package terminator not found yet:
      return;                    // cannot do anything now
    }
    buffer_[buf_i_++] = next_char; // other characters are added to the buffer
    if(buf_i_ >= BUFFER_SIZE){ // Buffer overflow. Strategy: flush buffer and start new. (This should not happen normally)
      buf_i_ = 0;
      return;
    }
  }

  Serial.print(buffer_);

  // this point is only reached when a Terminator symbol was reached
  if(buf_i_<4){buf_i_ = 0; return;} // skip too small packages //buffer flush 

  int package_start = 0;
  bool package_located = false;

  for (package_start = 0; package_start < buf_i_ - 2 && !package_located; package_start++) {
    if (buffer_[package_start] == 'A' && buffer_[package_start + 1] == 'T' && buffer_[package_start + 2] == ' ') {
      package_located = true;
      break; // explicitly break, otherwise package_start++ is executed once again
    }
  }

  if(!package_located){ // terminator symbol was reached but no package was located
    buf_i_ = 0;  // flush buffer
    return;
  }

  Serial.print(buffer_);

  // this point is only reached when package was successfully located
  
  int cur_i_cmd = package_start + 3;
  while (cur_i_cmd < buf_i_) {
    cur_cmd = buffer_[cur_i_cmd];
Serial.print(" ");
Serial.print(cur_cmd);
Serial.print(" ");
    long new_value = 0;
    if (cur_cmd == CMD_X_NEW_POS ||
        cur_cmd == CMD_Y_NEW_POS ||
        cur_cmd == CMD_Z_NEW_POS ||
        cur_cmd == CMD_SET_SPEED ||
        cur_cmd == CMD_SET_ACCEL) {
      if(sscanf (buffer_ + (cur_i_cmd + 1),"%ld",&new_value)<=0){buf_i_ = 0; return;} // flush and return if parsing error
    }
    switch (cur_cmd) {
      case CMD_X_NEW_POS:
        set_new_pos(-new_value, motor_X);
        break;
      case CMD_Y_NEW_POS:
        set_new_pos(-new_value, motor_Y);
        break;
      case CMD_Z_NEW_POS:
        set_new_pos(-new_value, motor_Z);
        break;
      case CMD_OPEN:
        if(!open_gripper){
          open_gripper = true;
          closed_gripper = false;
          set_new_pos(motor_A.currentPosition()+120,motor_A);
        } else {
          send_idle();
          send_idle();
        }
        break;
      case CMD_CLOSE:
        if(!closed_gripper){
          open_gripper = false;
          closed_gripper = true;
          set_new_pos(motor_A.currentPosition()-120,motor_A);
        } else {
          send_idle();
          send_idle();
        }
        break;
      case CMD_STATUS_REQ:
        if (cur_status == STATUS_IDLE) {
             send_idle();
        } else if (cur_status == STATUS_MOVING) {
             send_moving();
        } else if (cur_status == STATUS_ERROR) {
             send_error();
        }
        break;
      case CMD_CALIBRATE:
        calibrate();
        break;
      case CMD_SET_SPEED:
        set_new_speed(new_value);
        break;
      case CMD_SET_ACCEL:
        set_new_acc(new_value);
        break;
      default:
        //#ifdef DEBUG
           send_packet(STATUS_ERROR, 15);
        //#endif
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

void setup() {
  Serial.begin(115200);
  //Serial.setTimeout(0);

  // initialize the LIMIT_PIN as an input per motor:
  pinMode(MOTOR_X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_Z_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A_OPEN_LIMIT_PIN, INPUT_PULLUP);

  motor_X.setEnablePin(MOTOR_X_ENABLE_PIN);
  motor_X.setPinsInverted(false, false, true);
  motor_X.setMaxSpeed(DEFAULT_MAX_SPEED_X);
  motor_X.setAcceleration(DEFAULT_MAX_ACCEL_X);

  motor_Y.setEnablePin(MOTOR_Y_ENABLE_PIN);
  motor_Y.setPinsInverted(false, false, true);
  motor_Y.setMaxSpeed(DEFAULT_MAX_SPEED_Y);
  motor_Y.setAcceleration(DEFAULT_MAX_ACCEL_Y);

  motor_Z.setEnablePin(MOTOR_Z_ENABLE_PIN);
  motor_Z.setPinsInverted(false, false, true);
  motor_Z.setMaxSpeed(DEFAULT_MAX_SPEED_Z);
  motor_Z.setAcceleration(DEFAULT_MAX_ACCEL_Z);

  motor_A.setEnablePin(MOTOR_A_ENABLE_PIN);
  motor_A.setPinsInverted(false, false, true);
  motor_A.setMaxSpeed(DEFAULT_MAX_SPEED_A);
  motor_A.setAcceleration(DEFAULT_MAX_ACCEL_A);

  Serial.println("AT HELLO");
  set_status(STATUS_IDLE);

  TIMSK2 = (TIMSK2 & B11111110) | 0x01;  //enable interrupt of timer overflow
  TCCR2B = (TCCR2B & B11111000) | 0x02;  // set prescaler for Timer 2 to 32 -> 2us per cnt, 512us per overflow //TODO: LOWER THIS

  interrupts();

}

void loop() {
  if(movement_done_flag) {
    motor_X.disableOutputs(); // Since all motors share the enable pin, disabling one is sufficient
    movement_done_flag = false;
    set_status(STATUS_IDLE);
  }
  read_package();
}

ISR(TIMER2_OVF_vect) // this is called every overflow of the timer 2
{
  static bool occupied = false;
  if(occupied) return; //this interrupt is already called
  occupied = true;
  interrupts(); // we need interrupts here to catch all the incoming serial data
  if (cur_status == STATUS_MOVING) {
        bool movement_done = true;
        movement_done &= !motor_X.run();
        movement_done &= !motor_Y.run();
        movement_done &= !motor_Z.run();
        movement_done &= !motor_A.run();
        movement_done_flag = movement_done;
  }
  occupied = false;
}
