#include <Wire.h>
#include <AccelStepper.h>

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

#define MOTOR_XYZ_DIR_INV_MASK ~((1 << MOTOR_X_DIR_SHIFT)|(1 << MOTOR_Y_DIR_SHIFT)|(1 << MOTOR_Z_DIR_SHIFT))
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


AccelStepper motor_X(MOTOR_X_STEP_SHIFT, MOTOR_X_DIR_SHIFT);
AccelStepper motor_Y(MOTOR_Y_STEP_SHIFT, MOTOR_Y_DIR_SHIFT);
AccelStepper motor_Z(MOTOR_Z_STEP_SHIFT, MOTOR_Z_DIR_SHIFT);
AccelStepper motor_A(MOTOR_A_STEP_SHIFT, MOTOR_A_DIR_SHIFT);

#define AT "AT "
#define TERMINATOR '+'

#define CMD_CALIBRATE 'C'

#define CMD_X_NEW_POS 'X'
#define CMD_Y_NEW_POS 'Y'
#define CMD_Z_NEW_POS 'Z'
#define CMD_A_NEW_POS 'A'
#define CMD_OPEN 'O'
#define CMD_CLOSE 'G'
#define CMD_STATUS_REQ 'S'
#define CMD_SET_ACCEL '7'
#define CMD_SET_SPEED '9'
#define CMD_STOP '.'
#define CMD_FAST_STOP ':'

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

char status_array_[] = {'M', 'I', 'E'};

bool movement_done_flag = false;

bool open_gripper = false;
bool closed_gripper = false;

int steps_pending_X = 0;
int steps_pending_Y = 0;
int steps_pending_Z = 0;
int steps_pending_A = 0;

int cur_cmd = 0;

int cur_status = STATUS_IDLE;

#define BUFFER_SIZE 128
char buffer_[BUFFER_SIZE];
byte buf_i_ = 0;
String errormessage;

int button_x_state = 0; // limit_x status
int button_y_state = 0; // limit_y status
int button_z_state = 0; // limit_z status

void enable_step_interrupt() {
  TIMSK0 = 0x02;  //enable interrupt of timer overflow
}

void disable_step_interrupt() {
  TIMSK0 = 0x0;  //disable interrupt of timer overflow
}


void send_packet(int status_, int value_to_send) {
    Serial.print(AT);
    Serial.print(status_array_[status_]);
    Serial.print(value_to_send);
    Serial.print("\r\n");
}

void send_status() {
  Serial.print(AT);
  Serial.print(status_array_[cur_status]);
  Serial.print(" ");
  if(cur_status == STATUS_ERROR){
    Serial.print(errormessage);
  } else { // send all the information while moving and while idle
    Serial.print(-motor_X.currentPosition());
    Serial.print(" ");
    Serial.print(-motor_Y.currentPosition());
    Serial.print(" ");
    Serial.print(-motor_Z.currentPosition());
    Serial.print(" ");
    Serial.print(motor_A.currentPosition());
    Serial.print(" ");
    send_gripper_status();
  }
  Serial.print("\r\n");
}

void send_gripper_status()
{
  byte open_button = digitalRead(MOTOR_A_OPEN_LIMIT_PIN);
  if(open_button == LOW){
        Serial.print("OPEN");
  }
  if(open_button == HIGH){
        Serial.print("CLOSED");
  }
}

void set_status(int status_) {
  if (cur_status != status_) {
    cur_status = status_;
    send_status();
  }
}

// move motor to according end stop - dir is direction, (1 or -1)
void move_to_end_stop(int limit_pin, AccelStepper &motor, int dir) {
  bool button_state;
  movement_done_flag = false; 
  noInterrupts();
  motor.move(1000000 * dir);
  interrupts();
  while ((button_state = digitalRead(limit_pin)) == HIGH) {
    if(movement_done_flag) // movement is done, but button not yet pressed, continue searching
    {
      movement_done_flag = false;
      noInterrupts();
      motor.move(10000000 * dir);
      interrupts();
    }
  }
  // now the button is pressed
  disable_step_interrupt(); // no steps anymore
  movement_done_flag = false;
  motor.setCurrentPosition(0L); // this also resets speed

  // move out of the end stop
  // TODO: configure
  motor.move(1000 * dir * -1);
  enable_step_interrupt();
  while(!movement_done_flag); // wait until movement done
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
  set_new_speed(new_speed, motor_X);
  set_new_speed(new_speed, motor_Y);
  set_new_speed(new_speed, motor_Z);
  set_new_speed(new_speed, motor_A);
  interrupts(); // activate interrupts again
}

inline void set_new_speed(long new_speed, AccelStepper &motor) {
  motor.setMaxSpeed(new_speed);
}

void set_new_acc(long new_acc) {
  noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
  set_new_acc(new_acc,motor_X);
  set_new_acc(new_acc,motor_Y);
  set_new_acc(new_acc,motor_Z);
  set_new_acc(new_acc,motor_A);
  interrupts(); // activate interrupts again
}

inline void set_new_acc(long new_acc, AccelStepper &motor) {
  motor.setAcceleration(new_acc);
}

// stop all the motors using the most recent acceleration values
void slow_stop_all() {
  noInterrupts();
  motor_X.stop();
  motor_Y.stop();
  motor_Z.stop();
  motor_A.stop();
  interrupts();
}

// crank up the acceleration values before stopping. Also block until stopping is done (should be normally be reasonable fast)
void fast_stop_all() {
  noInterrupts(); // stop stepping!
  float accs[4] = { motor_X.getAcceleration(),
                    motor_Y.getAcceleration(),
                    motor_Z.getAcceleration(),
                    motor_A.getAcceleration()};
  // now pimp the acceleration
  set_new_acc(DEFAULT_MAX_ACCEL_X,motor_X);
  set_new_acc(DEFAULT_MAX_ACCEL_Y,motor_Y);
  set_new_acc(DEFAULT_MAX_ACCEL_Z,motor_Z);
  set_new_acc(DEFAULT_MAX_ACCEL_A,motor_A);
  slow_stop_all(); // this will also enable interrupts, thus stepping starts again
  // all motors are stopped now
  movement_done_flag = false; // this flag could have been raised during parsing time.
  while(!movement_done_flag); // wait until stopping movement is done.
  noInterrupts();
  set_new_acc(accs[0],motor_X);
  set_new_acc(accs[1],motor_Y);
  set_new_acc(accs[2],motor_Z);
  set_new_acc(accs[3],motor_A);
  interrupts();
  //go on like nothin' happened
}

void read_package() {
  char next_char;
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

  // this point is only reached when a Terminator symbol was reached
  if(buf_i_<4){buf_i_ = 0; return;} // skip too small packages //buffer flush 

  byte package_start = 0;
  bool package_located = false;

  for (package_start = 0; package_start < buf_i_ - 2; package_start++) {
    if (buffer_[package_start] == 'A' && buffer_[package_start + 1] == 'T' && buffer_[package_start + 2] == ' ') {
      package_located = true;
      break; // explicitly break, otherwise package_start++ is executed once again
    }
  }

  if(!package_located){ // terminator symbol was reached but no package was located
    buf_i_ = 0;  // flush buffer
    return;
  }

  // this point is only reached when package was successfully located
  
  byte cur_i_cmd = package_start + 3;
  while (cur_i_cmd < buf_i_) {
    char cur_cmd = buffer_[cur_i_cmd];
    long new_value = 0;
    if (cur_cmd == CMD_X_NEW_POS ||
        cur_cmd == CMD_Y_NEW_POS ||
        cur_cmd == CMD_Z_NEW_POS ||
        cur_cmd == CMD_A_NEW_POS ||
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
      case CMD_A_NEW_POS:
        set_new_pos(new_value, motor_A);
        break;
      case CMD_OPEN:
        if(!open_gripper){
          open_gripper = true;
          closed_gripper = false;
          set_new_pos(motor_A.currentPosition()+120,motor_A);
        } else {
          send_status();
          send_status();
        }
        break;
      case CMD_CLOSE:
        if(!closed_gripper){
          open_gripper = false;
          closed_gripper = true;
          set_new_pos(motor_A.currentPosition()-120,motor_A);
        } else {
          send_status();
          send_status();
        }
        break;
      case CMD_STATUS_REQ:
        send_status();
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
      case CMD_STOP:
        slow_stop_all();
        break;
      case CMD_FAST_STOP:
        fast_stop_all();
        break;
      default:
        #ifdef DEBUG
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

void setup() {
  Serial.begin(115200);
  //Serial.setTimeout(0);

  TIMSK0 = 0; // turn off interrupts on Timer 0. micros() will thus NOT work anymore!
  /* Why use timer 0 for the step interrupt?
  *  Why use timer 2 for the pule interrupts?
  *  By this procedure it's made sure that the pulse interrupts fire BEFORE a new step interrupt is called, in case both are pending
  */

  TCCR1A = 0x0;
  TCCR1B = 0x3; // use 64 prescaler -> 4 us per cnt
  TIMSK1 = 0x0; //deactivate all interrupts with this timer

  // initialize the LIMIT_PIN as an input per motor:
  pinMode(MOTOR_X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_Z_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A_OPEN_LIMIT_PIN, INPUT_PULLUP);

  pinMode(MOTOR_X_STEP_PIN, OUTPUT);
  pinMode(MOTOR_Y_STEP_PIN, OUTPUT);
  pinMode(MOTOR_Z_STEP_PIN, OUTPUT);
  pinMode(MOTOR_A_STEP_PIN, OUTPUT);

  motor_X.setEnablePin(MOTOR_X_ENABLE_PIN, true);
  set_new_speed(DEFAULT_MAX_SPEED_X, motor_X);
  set_new_acc(DEFAULT_MAX_ACCEL_X, motor_X);

  motor_Y.setEnablePin(MOTOR_Y_ENABLE_PIN, true);
  set_new_speed(DEFAULT_MAX_SPEED_Y, motor_Y);
  set_new_acc(DEFAULT_MAX_ACCEL_Y, motor_Y);

  motor_Z.setEnablePin(MOTOR_Z_ENABLE_PIN, true);
  set_new_speed(DEFAULT_MAX_SPEED_Z, motor_Z);
  set_new_acc(DEFAULT_MAX_ACCEL_Z, motor_Z);

  motor_A.setEnablePin(MOTOR_A_ENABLE_PIN, true);
  set_new_speed(DEFAULT_MAX_SPEED_A, motor_A);
  set_new_acc(DEFAULT_MAX_ACCEL_A, motor_A);

  Serial.println("AT HELLO");
  set_status(STATUS_IDLE);
  motor_X.disableOutputs();

  // configure the pulse interrupt

  TCCR2A = 0x1; // just normal mode
  TCCR2B = 0x0; //no clock source, activate with TCCR2B=0x1; just direct io clock
  TCNT2 = 0; // reset counter
  OCR2A = 15; // start pulse, should be at least 650ns after setting direction 
  OCR2B = 50; // end pulse, should be at least 1.9us after starting pulse
  TIFR2 = 0x7; // clear already set flags
  TIMSK2 = 0x6; // activate both compare interrupts

  // configure the step interrupt
 
  TCCR0A = 0x2; // CTC mode
  TCCR0B = 0x2; // 0.5us per cnt, prescaler is 8
  OCR0A = 250; // 125us per step
  TCNT0 = 0;
  TIMSK0 = 0; // start new
  TIFR0 = 0x7; // clear all interrupt flags
  enable_step_interrupt();

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

volatile byte step_bits_xyz = 0;
volatile byte step_bits_a = 0;
volatile bool pulse_done = true;


ISR(TIMER0_COMPA_vect) // this is called every overflow of the timer 2
{
  static bool occupied = false;
  if(occupied) return; //this interrupt is already called
  occupied = true;
  interrupts(); // we need interrupts here to catch all the incoming serial data and finish the pulse
  if (cur_status == STATUS_MOVING) {
    byte dir=0, step=0, step_a=0, dir_a=0; // using these allows for more efficient code, compared to using the above volatile corresponding step_bits_*
    motor_X.get_step(step, dir);
    motor_Y.get_step(step, dir);
    motor_Z.get_step(step, dir);
    motor_A.get_step(step_a, dir_a);
    
    if(step | step_a){ // only step if really necessary
      while(!pulse_done); // wait until the previous pulse is done // TODO:: remove after ensuring pulse is surely done here EVERY TIME!
      //Serial.println(dir,BIN);
      MOTOR_XYZ_DIR_OUT = (MOTOR_XYZ_DIR_OUT & MOTOR_XYZ_DIR_INV_MASK) | dir; // set the direction pins right
      MOTOR_A_DIR_OUT = (MOTOR_A_DIR_OUT & MOTOR_A_DIR_INV_MASK) | dir_a; // set the direction pins right
      step_bits_xyz = step;
      step_bits_a = step_a;

      pulse_done = false;
      TCCR2B=0x1; // activate pulse interrupts // GO GO GO
    }

    bool
    movement_done = motor_X.update_speed();
    movement_done &= motor_Y.update_speed();
    movement_done &= motor_Z.update_speed();
    movement_done &= motor_A.update_speed();
    movement_done_flag = movement_done;
  } 
  occupied = false;
}

ISR(TIMER2_COMPA_vect) // start of pulse
{
  MOTOR_XYZ_STEP_OUT |= step_bits_xyz;
  MOTOR_A_STEP_OUT |= step_bits_a;
}

ISR(TIMER2_COMPB_vect) // end of pulse
{
  MOTOR_XYZ_STEP_OUT &= ~step_bits_xyz; // stopping the pulse has higher priority than turning off the timer
  MOTOR_A_STEP_OUT &= ~step_bits_a;
  TCCR2B = 0x0;  // stop timer
  TCNT2 = 0;  // reset cnt value
  pulse_done = true;
}
