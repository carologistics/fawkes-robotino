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
#define CMD_A_NEW_POS 'A'

#define DEFAULT_MAX_SPEED_X 40000
#define DEFAULT_MAX_ACCEL_X 50000

#define DEFAULT_MAX_SPEED_Y 20000
#define DEFAULT_MAX_ACCEL_Y 50000

#define DEFAULT_MAX_SPEED_Z 20000
#define DEFAULT_MAX_ACCEL_Z 50000

#define DEFAULT_MAX_SPEED_A 20000
#define DEFAULT_MAX_ACCEL_A 50000

//#define CMD_SET_ACCEL 7
#define CMD_SET_SPEED 9

#define STATUS_MOVING 0
#define STATUS_IDLE 1
#define STATUS_ERROR 2

char status_array_[] = {'M', 'I', 'E'};

int steps_pending_X = 0;
int steps_pending_Y = 0;
int steps_pending_Z = 0;
int steps_pending_A = 0;

int cur_cmd = 0;

int cur_status = STATUS_IDLE;

char buffer_[128];
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
      motor.moveTo(10000000 * dir);
      motor.setSpeed(3000);
      motor.runSpeed();
    }
  }

  motor.setCurrentPosition(0L);

  // move out of the end stop
  // TODO: configure
  motor.move(3000 * dir * -1);
  while (motor.distanceToGo() != 0) {
 //   motor.setSpeed(DEFAULT_MAX_SPEED);
    motor.run();
  }

//  motor.setMaxSpeed(motor_speed);
}

bool calibrate_axis(int limit_pin, AccelStepper &motor) {
  int button_state = digitalRead(limit_pin);
  if (button_state == LOW) {
    //ERROR: We can't calibrate an axis when the end stop is already pressed!
    return false;
  }
  motor.enableOutputs();

  move_to_end_stop(limit_pin, motor, -1);

  // we found the starting point - move to the other end of the axis to get
  // the number of steps to the other end.

  //move_to_end_stop(limit_pin, motor, 1);
  motor.disableOutputs();
  return true;
}


// maybe change this to home all 3 axis simultaneously
void calibrate() {
  set_status(STATUS_MOVING);

  if (!calibrate_axis(MOTOR_X_LIMIT_PIN, motor_X)) {
    errormessage = "Can't calibrate axis X - end stop pressed!";
    set_status(STATUS_ERROR);
    return;
  }
  if (!calibrate_axis(MOTOR_Y_LIMIT_PIN, motor_Y)) {
    errormessage = "Can't calibrate axis Y - end stop pressed!";
    set_status(STATUS_ERROR);
    return;
  }
  if (!calibrate_axis(MOTOR_Z_LIMIT_PIN, motor_Z)) {
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
  motor.moveTo(new_pos);
  set_status(STATUS_MOVING);
}

void read_package() {
    int len = Serial.readBytesUntil(TERMINATOR, buffer_, 128);
    // Skip too short packages
    if (len < 4) return;

    int package_start = 0;
    bool package_located = false;

    for (package_start = 0; package_start < len - 2 && !package_located; package_start++) {
      if (buffer_[package_start] == 'A' && buffer_[package_start + 1] == 'T' && buffer_[package_start + 2] == ' ') {
        package_located = true;
      }
    }
    if (package_located) {
      len = len - package_start; // setup index to match starting of commands
      int cur_i_cmd = package_start;
      while (cur_i_cmd < len) {
        cur_cmd = buffer_[cur_i_cmd + 2];

        int new_pos = 0;
        int new_accel = 0;
        int new_speed = 0;
        if (cur_cmd == 'X' ||
            cur_cmd == 'Y' ||
            cur_cmd == 'Z' ||
            cur_cmd == 'A') {
          sscanf (buffer_ + (cur_i_cmd + 3),"%d",&new_pos);
          /*
          set_new_pos(new_pos, steppers[cur_cmd - 1]);
          Serial.print("read command: ");
          Serial.println(cur_cmd);
          */
        }
          switch (cur_cmd) {

            case CMD_X_NEW_POS:
              set_new_pos(new_pos, motor_X);
              break;
            case CMD_Y_NEW_POS:
              set_new_pos(new_pos, motor_Y);
              break;
            case CMD_Z_NEW_POS:
              set_new_pos(new_pos, motor_Z);
              break;
            case CMD_A_NEW_POS:
              set_new_pos(new_pos, motor_A);
              break;
            case CMD_CALIBRATE:
              calibrate();
              break;
            case CMD_SET_SPEED:
              sscanf (buffer_ + (cur_i_cmd + 3),"%d",&new_speed);
              motor_X.setMaxSpeed(new_speed);
              motor_Y.setMaxSpeed(new_speed);
              motor_Z.setMaxSpeed(new_speed);
              motor_A.setMaxSpeed(new_speed);
              break;
            default:
              #ifdef DEBUG
                 send_packet(STATUS_ERROR, "unknown command", 15);
              #endif
              break;
          }

        // move to next command
        while (cur_i_cmd < len) {
          cur_i_cmd += 1;
          if (buffer_[cur_i_cmd + 2] == ' ') {
            cur_i_cmd += 1;
            break;
          }
        }
      }

//      cur_cmd = buffer_[package_start + 2] - '0';

    }
    memset(buffer_, 0, 128);
    Serial.flush();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  // initialize the LIMIT_PIN as an input per motor:
  pinMode(MOTOR_X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_Z_LIMIT_PIN, INPUT_PULLUP);

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

}
void loop() {
  if (motor_X.distanceToGo() != 0 ||
      motor_Y.distanceToGo() != 0 ||
      motor_Z.distanceToGo() != 0 ||
      motor_A.distanceToGo() != 0) {

        motor_X.run();
        motor_Y.run();
        motor_Z.run();
        motor_A.run();


      } else if (cur_status == STATUS_MOVING) {
        // disable motors when we were still moving and send idle
        motor_X.disableOutputs();
        motor_Y.disableOutputs();
        motor_Z.disableOutputs();
        motor_A.disableOutputs();
        set_status(STATUS_IDLE);
      } else {
        read_package();
      }
}
