#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Connect a stepper motor with 48 steps per revolution (7.5 degree)
// to motor port #2 (M3 and M4)
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);

#define AT "AT+"
#define TERMINATOR 'X'
#define BUTTONPIN 42

// Rotation direction for upwards or downwards movement
#define UPWARDS BACKWARD
#define DOWNWARDS FORWARD

#define CMD_STEP_UP 1
#define CMD_STEP_DOWN 2
#define CMD_TO_UPPER_LIMIT 3

//#define DEBUG

int incomingByte = 0;   // for incoming serial data
int steps_pending = 0;
bool upwards = false;
bool to_start_pos_pending = false;
int cur_cmd = 0;

char buffer_[128];
int buffer_p = 0;
int buttonState = 0;         // variable for reading the pushbutton status


/*
void send_packet(const char* buffer_, int len) {
  Serial.print(AT);
  for (int i = 0; i < len; i++) {
    Serial.print(buffer_[i]);
  }
  Serial.print("\r\n");
}
*/

void gotoUpperLimit() {
  buttonState = digitalRead(BUTTONPIN);
  while (buttonState == LOW) {
      myMotor->step(1, UPWARDS, DOUBLE);
      delay(10);
      buttonState = digitalRead(BUTTONPIN);
  }
}

void read_package() {
//  if (Serial.available() > 0) {
    int len = Serial.readBytesUntil(TERMINATOR, buffer_, 128);
    // Skip too short packages
    if (len < 4) return;

//    Serial.print("I received: ");
//    Serial.print(len);
//    Serial.println(" Bytes:");

    int package_start = 0;
    bool package_located = false;

    for (package_start = 0; package_start < len - 2 && !package_located; package_start++) {
      if (buffer_[package_start] == 'A' && buffer_[package_start + 1] == 'T' && buffer_[package_start + 2] == '+') {
        package_located = true;
      }
    }
    if (package_located) {

      cur_cmd = buffer_[package_start + 2] - '0';
      int n_steps = 0;
      switch (cur_cmd) {
        case CMD_STEP_UP:
          #ifdef DEBUG
             Serial.println("step up");
          #endif
          n_steps = 0;
          sscanf (buffer_ + (package_start + 3),"%d",&n_steps);
          steps_pending = n_steps;
          upwards = true;
          break;
        case CMD_STEP_DOWN:
          #ifdef DEBUG
             Serial.println("step down");
          #endif
          n_steps = 0;
          sscanf (buffer_ + (package_start + 3),"%d",&n_steps);
          steps_pending = n_steps;
          upwards = false;
          break;
        case CMD_TO_UPPER_LIMIT:
          gotoUpperLimit();
          break;
        default:
          #ifdef DEBUG
             Serial.println("unknown command");
          #endif
          break;
      }
    }
    Serial.flush();
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.setTimeout(10);
  pinMode(13, OUTPUT);
  AFMS.begin();  // create with the default frequency 1.6KHz

  myMotor->setSpeed(500);  // 10 rpm
  // initialize the pushbutton pin as an input:
  pinMode(BUTTONPIN, INPUT);
}

void loop() {
  read_package();

  if (steps_pending != 0) {
    if (upwards) {
      myMotor->step(steps_pending, UPWARDS, DOUBLE);
      steps_pending = 0;
    } else {
      myMotor->step(steps_pending, DOWNWARDS, DOUBLE);
      steps_pending = 0;
    }
    myMotor->release();
  }
}
