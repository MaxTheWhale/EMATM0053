/*
       @@@@@@@@@@@&*           %@@@@@%       @@@@@@@@@    @@@@@@@@@  @@@@@@@@
       @@@@@@@@@@@@@@@     #@@@@@@@@@@@@    @@@@@@@@@@   @@@@@@@@@* @@@@@@@@@
       @@@@@@   @@@@@@   /@@@@@%  .@@@@@@    @@@/@@@@@ @@@@@@@@@@    @@@@@@
      &@@@@@##&@@@@@@   @@@@@@(   @@@@@@@   @@@,.@@@@@@@@,.@@@@@    @@@@@@
      @@@@@@@@@@@@@    &@@@@@@    @@@@@@   @@@@  @@@@@@@  @@@@@    (@@@@@
     @@@@@@  @@@@@@*   @@@@@@    @@@@@@   .@@@   @@@@@#  @@@@@@    @@@@@&
   @@@@@@@@   @@@@@@%  .@@@@@@@@@@@@@    @@@@@%  @@@@  @@@@@@@@  @@@@@@@@
  %@@@@@@@&   @@@@@@     #@@@@@@@@      @@@@@@   @@@   @@@@@@@/ @@@@@@@@%

  Provided by Paul O'Dowd Oct 2020
*/


// The following files should all appear in
// tabs above.  They are incomplete and match
// up to exercie labsheets provided.
#include "lineSensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "motor.h"
#include "pid.h"

// You may need to change these depending on how you wire
// in your line sensor.
#define LINE_LEFT_PIN A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A2 //Pin for the right line sensor

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

lineSensor_c line_left( LINE_LEFT_PIN ); //Create a line sensor object for the left sensor
lineSensor_c line_centre( LINE_CENTRE_PIN ); //Create a line sensor object for the centre sensor
lineSensor_c line_right( LINE_RIGHT_PIN ); //Create a line sensor object for the right sensor

motor_c left_motor( L_PWM_PIN, L_DIR_PIN );
motor_c right_motor( R_PWM_PIN, R_DIR_PIN );

// Experiment with your gains slowly, one by one.
float Kp_left = 0.05f; //Proportional gain 
float Kd_left = -0.3f; //Derivative gain
float Ki_left = 0.003f; //Integral gain
PID_c left_PID(Kp_left, Ki_left, Kd_left); // controller for left wheel

// Experiment with your gains slowly, one by one.
float Kp_right = 0.045f; //Proportional gain 
float Kd_right = -0.3f; //Derivative gain
float Ki_right = 0.003f; //Integral gain
PID_c right_PID(Kp_right, Ki_right, Kd_right); // controller for right wheel

// Experiment with your gains slowly, one by one.
float Kp_heading = 400.0f; //Proportional gain 
float Kd_heading = 0.0f; //Derivative gain
float Ki_heading = 0.0f; //Integral gain
PID_c heading_PID(Kp_heading, Ki_heading, Kd_heading); // controller for heading

kinematics_c pose;

long prev_loop_left_count, prev_loop_right_count;
unsigned long timestamp;
float loop_left_speed, loop_right_speed;

// States
#define STATE_INITIAL           0
#define STATE_DRIVE_FORWARDS    1
#define STATE_FOLLOW_LINE       2
#define STATE_CHECK_RIGHT_ANGLE 3
#define STATE_RETURN_HOME       4
#define STATE_DONE              5

int state;

#define PID_UPDATE_PERIOD 100
#define SPEED_UPDATE_PERIOD 50
#define KINEMATICS_UPDATE_PERIOD 50

unsigned long pid_update_millis = 0;
unsigned long speed_update_millis = 0;
unsigned long kinematics_update_millis = 0;
float demand_left, demand_right;

float desired_angle = M_PI - 0.1f;
float target_x = 5000.0f;
float target_y = 2500.0f;

#define HEADING_NONE        0
#define HEADING_LINE_SENSOR 1
#define HEADING_ANGLE       2

int heading_measurement;

#define NO_TURN        0
#define TURNING_RIGHT  1
#define TURNING_LEFT   2
#define TURNING_CENTRE 3

int right_angle_state;
float original_theta;

#define LOST_LINE_THRESHOLD 3
int lost_line_count;

bool line_break_crossed;
bool stopped;

// Setup, only runs once when the power
// is turned on.  However, if your Romi
// gets reset, it will run again.
void setup() {

  // Start up the serial port.
  Serial.begin(9600);

  // Delay to connect properly.
  delay(1000);

  // Print a debug, so we can see a reset on monitor.
  Serial.println("***RESET***");

  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();

  setupEncoder1();
  setupEncoder0();
  setupTimer3();

  prev_loop_left_count = 0;
  prev_loop_right_count = 0;
  loop_left_speed = 0.0f;
  loop_right_speed = 0.0f;
  timestamp = 0;

  demand_left = 0.0f;
  demand_right = 0.0f;

  state = STATE_INITIAL;
  heading_measurement = HEADING_NONE;

  lost_line_count = 0;

  line_break_crossed = false;
  stopped = false;
}

#define ON_LINE_THRESHOLD 100
#define TURN_POWER 50.0f
#define FORWARD_POWER 20.0f

float confidence = 0.0f;

void BangBang() {
  int left = line_left.read();
  int centre = line_centre.read();
  int right = line_right.read();
  float total = left + right + centre;
  float m = 0;
  int l_power = 0;
  int r_power = 0;
  if (line_centre.onLine()) {
    digitalWrite(LED_BUILTIN, HIGH);
    float l_norm = left / total;
    float c_norm = centre / total;
    float r_norm = right / total;
    m = l_norm - r_norm;
    int l_turn_bias = (int)(TURN_POWER * m);
    int r_turn_bias = (int)(-TURN_POWER * m);
    int forward_bias = (int)(FORWARD_POWER * (1.0f - abs(m)) * confidence);
    l_power = l_turn_bias + forward_bias;
    r_power = r_turn_bias + forward_bias;
      // m = 1, l = 20, r = -20
      // m = 0, l = 20, r = 20
      // m = -1,l = -20,  r = 20
      // bias is inversely proportional to m
      // l_power = l_turn_bias + forward_bias
      // l_turn_bias = TURN_POWER * m
      // forward_bias = FORWARD_POWER * (1 - abs(m))
    if (confidence < 1.0f) confidence += 0.01f;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    confidence = 0.0f;
    l_power = 20;
    r_power = 20;
  }
  Serial.println(m);
  Serial.println(l_power);
  Serial.println(r_power);
  Serial.println();


  left_motor.setPower(l_power);
  right_motor.setPower(r_power);
}

float calculate_m() {
  int left = line_left.read();
  int centre = line_centre.read();
  int right = line_right.read();
  float total = left + right + centre;
  float m = 0;
  if (total > 100) {
    float l_norm = left / total;
    float r_norm = right / total;
    m = r_norm - l_norm;
    if (m < -1.0f || m > 1.0f) Serial.println("calculate_m: bad m value");
  }

  return m;
}

void stop() {
  demand_left = 0.0f;
  demand_right = 0.0f;
  left_PID.reset();
  right_PID.reset();
  stopped = true;
}

float wrap_angle(float theta) {
  while (theta > M_PI || theta < -M_PI) {
    if (theta > M_PI) theta -= 2.0f * M_PI;
    if (theta < -M_PI) theta += 2.0f * M_PI;
  }
  return theta;
}

// The main loop of execution.  This loop()
// function is automatically called every
// time it finishes.  You should try to write
// your code to take advantage of this looping
// behaviour.  
void loop() {
  unsigned long this_millis = millis();

  if (this_millis > speed_update_millis + SPEED_UPDATE_PERIOD) {
    unsigned int elapsed_time = micros() - timestamp;

    int left_change = left_count - prev_loop_left_count;
    int right_change = right_count - prev_loop_right_count;

    loop_left_speed = left_change / (float)elapsed_time * 1000000;
    loop_right_speed = right_change / (float)elapsed_time * 1000000;

    prev_loop_left_count = left_count;
    prev_loop_right_count = right_count;

    pose.update(left_change, right_change);

    Serial.print(pose.x);
    Serial.print(',');
    Serial.print(pose.y);
    Serial.print(',');
    Serial.println(pose.theta);

    timestamp = micros();
    speed_update_millis = this_millis;
  }

  if (this_millis > pid_update_millis + PID_UPDATE_PERIOD) {

    float turn_demand = 0.0f;
    if (heading_measurement == HEADING_ANGLE) {
      float dx = target_x - pose.x;
      float dy = target_y - pose.y;
      float required_angle = atan2f(dy, dx);
      turn_demand = heading_PID.update(required_angle, wrap_angle(pose.theta));
    } else if (heading_measurement == HEADING_LINE_SENSOR) {
      turn_demand = heading_PID.update(0, calculate_m());
    }

    if (!stopped) {
      float output_left = left_PID.update(demand_left - turn_demand, left_speed);
      float output_right = right_PID.update(demand_right + turn_demand, right_speed);

      left_motor.setPower(output_left);
      right_motor.setPower(output_right);
    } else {
      left_motor.setPower(0);
      right_motor.setPower(0);
    }

    confidence += 0.005f;

    if (!line_centre.onLine() && !line_left.onLine() && !line_right.onLine()) {
      lost_line_count += 1;
    } else {
      lost_line_count = 0;
    }

    pid_update_millis = this_millis;
  }

  if (this_millis > kinematics_update_millis + KINEMATICS_UPDATE_PERIOD) {
    

    kinematics_update_millis = this_millis;
  }

  // Based on the value of STATE variable,
  // run code for the appropriate robot behaviour.
  if ( state == STATE_INITIAL ) {
    state = STATE_DRIVE_FORWARDS;
  } else if ( state == STATE_DRIVE_FORWARDS ) {
    demand_left = 500.0f;
    demand_right = 500.0f;
    if (line_centre.onLine()) {
      state = STATE_FOLLOW_LINE;
      heading_measurement = HEADING_LINE_SENSOR;
      left_PID.reset();
      right_PID.reset();
      demand_left = 0.0f;
      demand_right = 0.0f;
      confidence = 0.0f;
    }
  } else if ( state == STATE_FOLLOW_LINE ) {
    bool left = line_left.onLine();
    bool centre = line_centre.onLine();
    bool right = line_right.onLine();

    digitalWrite(LED_BUILTIN, (centre) ? HIGH : LOW);
    digitalWrite(30, (left) ? LOW : HIGH);
    digitalWrite(17, (right) ? LOW : HIGH);
    if (!centre && !left && !right && lost_line_count >= LOST_LINE_THRESHOLD) {
      confidence = 0.0f;
      state = STATE_CHECK_RIGHT_ANGLE;
      right_angle_state = TURNING_RIGHT;
      original_theta = pose.theta;
      demand_left = 0.0f;
      demand_right = 0.0f;
      left_PID.reset();
      right_PID.reset();
    } else {
      demand_left = 300.0f * confidence + 200.0f;
      demand_right = 300.0f * confidence + 200.0f;
    }
  } else if ( state == STATE_CHECK_RIGHT_ANGLE ) {
    // Turn 90 degrees left, then 90 degrees right, then back to centre
    // If the line is seen at any point stop

    if (right_angle_state == TURNING_RIGHT) {
      demand_left = 500.0f;
      demand_right = -500.0f;

      if (abs((original_theta - M_PI_2) - pose.theta) < 0.1f) {
        right_angle_state = TURNING_LEFT;
      }
    } else if (right_angle_state == TURNING_LEFT) {
      demand_left = -500.0f;
      demand_right = 500.0f;

      if (abs((original_theta + M_PI_2) - pose.theta) < 0.1f) {
        right_angle_state = TURNING_CENTRE;
      }
    } else if (right_angle_state == TURNING_CENTRE) {
      demand_left = 500.0f;
      demand_right = -500.0f;

      if (abs(original_theta - pose.theta) < 0.1f) {
        if (line_break_crossed) {
          state = STATE_RETURN_HOME;
          heading_measurement = HEADING_ANGLE;
          left_PID.reset();
          right_PID.reset();
        } else {
          state = STATE_DRIVE_FORWARDS;
          left_PID.reset();
          right_PID.reset();
          confidence = 0.0f;
          heading_measurement = HEADING_NONE;
          line_break_crossed = true;
        }
      }
    }

    if (line_centre.onLine()) {
      state = STATE_FOLLOW_LINE;
      heading_measurement = HEADING_LINE_SENSOR;
      left_PID.reset();
      right_PID.reset();
      demand_left = 0.0f;
      demand_right = 0.0f;
      confidence = 0.0f;
    }
  } else if ( state == STATE_RETURN_HOME ) {
    target_x = 0.0f;
    target_y = 0.0f;

    float dx = target_x - pose.x;
    float dy = target_y - pose.y;
    float required_angle = atan2f(dy, dx);
    float angle_difference = abs(required_angle - wrap_angle(pose.theta));
    if (angle_difference < 0.2f) {
      demand_left = 500.0f;
      demand_right = 500.0f;
    } else {
      demand_left = 0.0f;
      demand_right = 0.0f;
    }
    if (abs(dx) + abs(dy) < 200.0f) {
      demand_left = 0.0f;
      demand_right = 0.0f;
      stopped = true;
      state = STATE_DONE;
      heading_measurement = HEADING_NONE;
    }
  } else if ( state == STATE_DONE ) {
    // Do nothing!
  } else {
    Serial.print("System Error, Unknown state: ");
    Serial.println(state);
  }
  
}
