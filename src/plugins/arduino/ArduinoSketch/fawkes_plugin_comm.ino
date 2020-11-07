#include <Wire.h>
#include <AccelStepper.h>

// #define DEBUG_MODE

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
AccelStepper motor_A(MOTOR_A_STEP_SHIFT, MOTOR_A_DIR_SHIFT);

long a_toggle_steps = 240;

#define AT "AT "
#define TERMINATOR '+'

#define CMD_CALIBRATE 'c'
#define CMD_DOUBLE_CALIBRATE 'C'

#define CMD_X_NEW_POS 'X'
#define CMD_Y_NEW_POS 'Y'
#define CMD_Z_NEW_POS 'Z'
#ifdef DEBUG_MODE
  #define CMD_A_NEW_POS 'A'
#endif
#define CMD_OPEN 'O'
#define CMD_CLOSE 'G'
#define CMD_STATUS_REQ 'S'
#define CMD_SET_ACCEL '7'
#define CMD_SET_SPEED '9'
#define CMD_STOP '.'
#define CMD_FAST_STOP ':'

#define CMD_A_SET_TOGGLE_STEPS 'T'

#define CMD_X_NEW_SPEED 'x'
#define CMD_Y_NEW_SPEED 'y'
#define CMD_Z_NEW_SPEED 'z'
#define CMD_A_NEW_SPEED 'a'

#define CMD_X_NEW_ACC 'm'
#define CMD_Y_NEW_ACC 'n'
#define CMD_Z_NEW_ACC 'o'
#define CMD_A_NEW_ACC 'p'

#define DEFAULT_MAX_SPEED_X 2000
#define DEFAULT_MAX_ACCEL_X 5000

#define DEFAULT_MAX_SPEED_Y 2000
#define DEFAULT_MAX_ACCEL_Y 3500

#define DEFAULT_MAX_SPEED_Z 2000
#define DEFAULT_MAX_ACCEL_Z 3500

#define DEFAULT_MAX_SPEED_A 7000
#define DEFAULT_MAX_ACCEL_A 15000

#define SECOND_CAL_MAX_SPEED 500
#define SECOND_CAL_MAX_ACC 1000


#define STATUS_MOVING 0
#define STATUS_IDLE 1
#define STATUS_ERROR 2

char status_array_[] = {'M', 'I', 'E'};

volatile bool movement_done_flag = false;

bool open_gripper = false;

int cur_status = STATUS_IDLE;

#define BUFFER_SIZE 128
char buffer_[BUFFER_SIZE];
byte buf_i_ = 0;
String errormessage;

void enable_step_interrupt() {
  TIMSK0 = 0x02;  // enable interrupt of timer overflow
}

void disable_step_interrupt() {
  TIMSK0 = 0x0;  // disable interrupt of timer overflow
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
  check_gripper_endstop();
  if(open_gripper)
    Serial.print("OPEN");
  else 
    Serial.print("CLOSED");
}

void check_gripper_endstop()
{
  byte open_button = digitalRead(MOTOR_A_OPEN_LIMIT_PIN);
  if(open_button == HIGH){ // definetely OPEN
    open_gripper = true;
  } else { // gripper should be closed
    open_gripper = false;
  }
}

void set_status(int status_) {
  if (cur_status != status_) {
    cur_status = status_;
    send_status();
  }
}

bool assumed_gripper_state;

// @Return True if gripper is assumed to be open
// This helper function is necessary to set the assumed_gripper_state initially
bool get_assumed_gripper_state(bool is_open_command) {
  static bool initialized = false;
  if(!initialized){
    initialized = true;
    assumed_gripper_state = !is_open_command;// assume closed if command is opening, assume open if command is closing
  }
  return assumed_gripper_state;
}

void double_calibrate()
{
  // first fast calibration run
  calibrate();
  while(!movement_done_flag);
  movement_done_flag = false;
  // reduce speed to a minimum
  float speeds[3] = {motor_X.get_speed(), motor_Y.get_speed(), motor_Z.get_speed()};
  float accs[3] = {motor_X.get_acc(), motor_Y.get_acc(), motor_Z.get_acc()};
  set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC,motor_X);
  set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC,motor_Y);
  set_new_speed_acc(SECOND_CAL_MAX_SPEED, SECOND_CAL_MAX_ACC,motor_Z);
  // calibrate a second time
  calibrate();
  while(!movement_done_flag);
  movement_done_flag = false;
  // after calibration the old speed and acc values are used
  set_new_speed_acc(speeds[0], accs[0], motor_X);
  set_new_speed_acc(speeds[1], accs[1], motor_Y);
  set_new_speed_acc(speeds[2], accs[2], motor_Z);
}

void calibrate()
{
  bool x_done = false, y_done = false, z_done = false;
  do { //repeat calibration as long as not successfull
    motor_X.enableOutputs();
    noInterrupts();
    if(!x_done) motor_X.move(20000L);
    if(!y_done) motor_Y.move(20000L);
    if(!z_done) motor_Z.move(20000L);
    movement_done_flag = false;
    interrupts();
    // due to high step count, reaching end stops is guaranteed!
    set_status(STATUS_MOVING); // status is always only changed on no interrupt code level, hence no race condition occurs here
    // This while loop controls permanently the state of the respective end stops and handles crashing into them
    // When all end stops are triggered simulatenously, additional latency is introduced.
    // The latency is maily due to the planning of the back movement.
    // One work around is to calibrate twice, the first time fast and the second time slowly.
    // (again inspired from grbl)
    // Additionally, the backwards movement should use different numbers of steps
    // to ensure that the end stops will not be triggerend simulatenously at the second calibration.
    while(!movement_done_flag && (!x_done || !y_done || !z_done)){
      if(!x_done && digitalRead(MOTOR_X_LIMIT_PIN)==LOW){ x_done=true; reach_end_handle(motor_X,0);}
      if(!y_done && digitalRead(MOTOR_Y_LIMIT_PIN)==LOW){ y_done=true; reach_end_handle(motor_Y,100);}
      if(!z_done && digitalRead(MOTOR_Z_LIMIT_PIN)==LOW){ z_done=true; reach_end_handle(motor_Z,200);}
    }
    movement_done_flag = false;
  } while(!x_done || !y_done || !z_done);
}


void reach_end_handle(AccelStepper &motor, byte extra){
  motor.hard_stop();
  motor.setCurrentPosition(0L);
  motor.move(-400-extra);
}

void set_new_pos(long new_pos, AccelStepper &motor) {
  motor.enableOutputs();
  noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
  motor.moveTo(new_pos);
  interrupts(); // activate interrupts again
  set_status(STATUS_MOVING); // status is always only changed on no interrupt code level, hence no race condition occurs here
}

void set_new_rel_pos(long new_rel_pos, AccelStepper &motor) {
  motor.enableOutputs();
  noInterrupts();
  motor.move(new_rel_pos);
  interrupts();
  set_status(STATUS_MOVING);
}

void set_new_speed(float new_speed) {
  noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
  set_new_speed_acc(new_speed, -1, motor_X);
  set_new_speed_acc(new_speed, -1, motor_Y);
  set_new_speed_acc(new_speed, -1, motor_Z);
  set_new_speed_acc(new_speed, -1, motor_A);
  interrupts(); // activate interrupts again
}

void set_new_acc(float new_acc) {
  noInterrupts(); // shortly disable interrupts to preverent stepping while changing target position (this is actually only a problem when cur_status == STATUS_MOVING)
  set_new_speed_acc(-1, new_acc,motor_X);
  set_new_speed_acc(-1, new_acc,motor_Y);
  set_new_speed_acc(-1, new_acc,motor_Z);
  set_new_speed_acc(-1, new_acc,motor_A);
  interrupts(); // activate interrupts again
}

inline void set_new_speed_acc(float new_speed, float new_acc, AccelStepper &motor)
{
   motor.setMaxSpeedAcc(new_speed, new_acc);
}

// stop all the motors using the most recent acceleration values
void slow_stop_all() {
  motor_X.stop();
  motor_Y.stop();
  motor_Z.stop();
  motor_A.stop();
}

// stop all motors with infinite acceleration. Do not use if step counter should still be correct afterwards.
void fast_stop_all() {
  motor_X.hard_stop();
  motor_Y.hard_stop();
  motor_Z.hard_stop();
  motor_A.hard_stop();
}

void read_package() {
  char next_char;
  while(true) 
  {
    next_char = Serial.read();
    if(next_char == TERMINATOR){ // if we find the terminator character we can analyze the package now
      buffer_[buf_i_] = 0; // Set null character to get no trouble with sscanf // buf_i_ points now onto 0
      break;
    } else if(next_char == -1) { // if no serial data is available anymore, but package terminator not found yet:
      return;                    // cannot do anything now
    }
    buffer_[buf_i_++] = next_char; // other characters are added to the buffer
    if(buf_i_ >= BUFFER_SIZE){ // Buffer overflow. Strategy: flush buffer and start new. (This should not happen normally)
      buf_i_ = 0;
      return;
    }
  }

  // this point is only reached when a Terminator symbol was reached
  if(buf_i_<4){buf_i_ = 0; return;} // skip too small packages // buffer flush 

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
        cur_cmd == CMD_A_SET_TOGGLE_STEPS ||
#ifdef DEBUG_MODE
        cur_cmd == CMD_A_NEW_POS ||
#endif
        cur_cmd == CMD_X_NEW_SPEED ||
        cur_cmd == CMD_Y_NEW_SPEED ||
        cur_cmd == CMD_Z_NEW_SPEED ||
        cur_cmd == CMD_A_NEW_SPEED ||
        cur_cmd == CMD_X_NEW_ACC ||
        cur_cmd == CMD_Y_NEW_ACC ||
        cur_cmd == CMD_Z_NEW_ACC ||
        cur_cmd == CMD_A_NEW_ACC ||
        cur_cmd == CMD_SET_SPEED ||
        cur_cmd == CMD_SET_ACCEL) {
      if(sscanf (buffer_ + (cur_i_cmd + 1),"%ld",&new_value)<=0){buf_i_ = 0; return;} // flush and return if parsing error
    }
    bool assumed_gripper_state_local; // this is used to store the assumed gripper state locally, to reduce calls to the function get_assumed_gripper_state
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
      case CMD_A_SET_TOGGLE_STEPS:
        a_toggle_steps = new_value;
        send_status();
        send_status();
        break;
#ifdef DEBUG_MODE
      case CMD_A_NEW_POS:
        set_new_pos(new_value, motor_A);
        break;
#endif
      case CMD_X_NEW_SPEED:
        set_new_speed_acc(new_value, 0.0, motor_X);
        send_status();
        send_status();
        break;
      case CMD_Y_NEW_SPEED:
        set_new_speed_acc(new_value, 0.0, motor_Y);
        send_status();
        send_status();
        break;
      case CMD_Z_NEW_SPEED:
        set_new_speed_acc(new_value, 0.0, motor_Z);
        send_status();
        send_status();
        break;
      case CMD_A_NEW_SPEED:
        set_new_speed_acc(new_value, 0.0, motor_A);
        send_status();
        send_status();
        break;
      case CMD_X_NEW_ACC:
        set_new_speed_acc(0.0, new_value, motor_X);
        send_status();
        send_status();
        break;
      case CMD_Y_NEW_ACC:
        set_new_speed_acc(0.0, new_value, motor_Y);
        send_status();
        send_status();
        break;
      case CMD_Z_NEW_ACC:
        set_new_speed_acc(0.0, new_value, motor_Z);
        send_status();
        send_status();
        break;
      case CMD_A_NEW_ACC:
        set_new_speed_acc(0.0, new_value, motor_A);
        send_status();
        send_status();
        break;
      case CMD_OPEN:
        check_gripper_endstop();
        assumed_gripper_state_local = get_assumed_gripper_state(true);
        if(!assumed_gripper_state_local && open_gripper || !open_gripper)
        { // we do it
          set_new_rel_pos(-a_toggle_steps,motor_A);
          assumed_gripper_state = true;
        } else { // we don't do it
          send_status();
          send_status();
        }
        break;
      case CMD_CLOSE:
        check_gripper_endstop();
        assumed_gripper_state_local = get_assumed_gripper_state(false);
        if(assumed_gripper_state_local)
        { // we do it
          set_new_rel_pos(a_toggle_steps,motor_A);
          assumed_gripper_state = false;
        } else { // we don't do it
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
      case CMD_DOUBLE_CALIBRATE:
        double_calibrate();
        break;
      case CMD_SET_SPEED:
        set_new_speed(new_value);
        send_status();
        send_status();
        break;
      case CMD_SET_ACCEL:
        set_new_acc(new_value);
        send_status();
        send_status();
        break;
      case CMD_STOP:
        slow_stop_all();
        break;
      case CMD_FAST_STOP:
        fast_stop_all();
        break;
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

void setup() {
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
  pinMode(MOTOR_A_STEP_PIN, OUTPUT);

  pinMode(MOTOR_X_DIR_PIN, OUTPUT);
  pinMode(MOTOR_Y_DIR_PIN, OUTPUT);
  pinMode(MOTOR_Z_DIR_PIN, OUTPUT);
  pinMode(MOTOR_A_DIR_PIN, OUTPUT);

  motor_X.setEnablePin(MOTOR_X_ENABLE_PIN, true);
  motor_Y.setEnablePin(MOTOR_Y_ENABLE_PIN, true);
  motor_Z.setEnablePin(MOTOR_Z_ENABLE_PIN, true);
  motor_A.setEnablePin(MOTOR_A_ENABLE_PIN, true);
  motor_X.disableOutputs(); // same pin for all of them

  set_new_speed_acc(DEFAULT_MAX_SPEED_X, DEFAULT_MAX_ACCEL_X, motor_X);
  set_new_speed_acc(DEFAULT_MAX_SPEED_Y, DEFAULT_MAX_ACCEL_Y, motor_Y);
  set_new_speed_acc(DEFAULT_MAX_SPEED_Z, DEFAULT_MAX_ACCEL_Z, motor_Z);
  set_new_speed_acc(DEFAULT_MAX_SPEED_A, DEFAULT_MAX_ACCEL_A, motor_A);


  Serial.println("AT HELLO");
  set_status(STATUS_IDLE);
  motor_X.disableOutputs();

  // configure the pulse interrupt

  TCCR2A = 0x1; // just normal mode
  TCCR2B = 0x0; //no clock source, activate with TCCR2B=0x1; just direct io clock
  TCNT2 = 0; // reset counter
  OCR2A = 3;//15; // start pulse, should be at least 650ns after setting direction 
  OCR2B = 254; // end pulse, should be at least 1.9us after starting pulse
  TIFR2 = 0x7; // clear already set flags
  TIMSK2 = 0x6; // activate both compare interrupts

  // configure the step interrupt
 
  TCCR0A = 0x2; // CTC mode
  TCCR0B = 0x2; // 0.5us per cnt, prescaler is 8
  OCR0A = 70; // 35us per step
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


ISR(TIMER0_COMPA_vect) // this is called every overflow of the timer 0
{
  static volatile bool occupied = false;
  if(occupied) return; // this interrupt is already called
  occupied = true;
  interrupts(); // we need interrupts here to catch all the incoming serial data and finish the pulse
  if (cur_status == STATUS_MOVING) {
    byte dir=0, step=0, step_a=0, dir_a=0; // using these allows for more efficient code, compared to using the above volatile corresponding step_bits_*
    bool movement_done;


    unsigned int time = CUR_TIME;
    movement_done = motor_X.get_step(step, dir, time);
    movement_done &= motor_Y.get_step(step, dir, time);
    movement_done &= motor_Z.get_step(step, dir, time);
    movement_done &= motor_A.get_step(step_a, dir_a, time);
    movement_done_flag = movement_done;
    if(step | step_a){ // only step if really necessary
      while(!pulse_done); // wait until the previous pulse is done // TODO:: remove after ensuring pulse is surely done here EVERY TIME!
      // Serial.println(dir,BIN);
      dir ^= 1 << MOTOR_X_DIR_SHIFT;
      dir ^= 1 << MOTOR_Y_DIR_SHIFT;
      MOTOR_XYZ_DIR_OUT = (MOTOR_XYZ_DIR_OUT & MOTOR_XYZ_DIR_INV_MASK) | dir; // set the direction pins right
      MOTOR_A_DIR_OUT = (MOTOR_A_DIR_OUT & MOTOR_A_DIR_INV_MASK) | dir_a; // set the direction pins right
      step_bits_xyz = step;
      step_bits_a = step_a;

      pulse_done = false;
      TCCR2B=0x1; // activate pulse interrupts // GO GO GO
    }

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
