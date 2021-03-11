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
float Kp_heading = 600.0f; //Proportional gain 
float Kd_heading = 0.0f; //Derivative gain
float Ki_heading = 0.0f; //Integral gain
PID_c heading_PID(Kp_heading, Ki_heading, Kd_heading); // controller for heading

kinematics_c pose;

long prev_loop_left_count, prev_loop_right_count;

// States
#define STATE_DRIVE_FORWARDS    0
#define STATE_FOLLOW_LINE       1
#define STATE_CHECK_RIGHT_ANGLE 2
#define STATE_UTURN             3
#define STATE_RETURN_HOME       4
#define STATE_DONE              5

int state;

#define PID_UPDATE_PERIOD 100
#define POSE_UPDATE_PERIOD 50

unsigned long pid_update_millis = 0;
unsigned long pose_update_millis = 0;
float demand_left, demand_right;

#define TURNING_RIGHT  0
#define TURNING_LEFT   1
#define TURNING_CENTRE 2

int right_angle_state;
float original_theta;

float confidence;

#define LOST_LINE_THRESHOLD 3
int lost_line_count;

float line_end_x;
float line_end_y;

bool crossing_line_break;
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

  demand_left = 0.0f;
  demand_right = 0.0f;

  confidence = 0.0f;

  state = STATE_DRIVE_FORWARDS;

  lost_line_count = 0;

  line_break_crossed = false;
  crossing_line_break = false;
  stopped = false;
}

float calculateM() {
  int left = line_left.read();
  int centre = line_centre.read();
  int right = line_right.read();
  float total = left + right + centre;
  float m = 0;
  if (total > 300) {
    float l_norm = left / total;
    float r_norm = right / total;
    m = r_norm - l_norm;
    if (m > 1.0f) m = 1.0f;
    if (m < -1.0f) m = -1.0f;
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

float wrapAngle(float theta) {
  while (theta > M_PI || theta < -M_PI) {
    if (theta > M_PI) theta -= 2.0f * M_PI;
    if (theta < -M_PI) theta += 2.0f * M_PI;
  }
  return theta;
}

float findAngleDifference(float theta) {
  float angle_difference = theta - pose.theta;
  if (angle_difference > M_PI) angle_difference = -2 * M_PI + angle_difference;
  if (angle_difference < -M_PI) angle_difference = 2 * M_PI + angle_difference;
}

void changeState(int new_state) {
  state = new_state;
  confidence = 0.0f;
  left_PID.reset();
  right_PID.reset();
}

// The main loop of execution.  This loop()
// function is automatically called every
// time it finishes.  You should try to write
// your code to take advantage of this looping
// behaviour.  
void loop() {
  unsigned long this_millis = millis();

  if (this_millis > pose_update_millis + POSE_UPDATE_PERIOD) {

    int left_change = left_count - prev_loop_left_count;
    int right_change = right_count - prev_loop_right_count;

    prev_loop_left_count = left_count;
    prev_loop_right_count = right_count;

    pose.update(left_change, right_change);

    Serial.print("Pose: ");
    Serial.print(pose.x);
    Serial.print(',');
    Serial.print(pose.y);
    Serial.print(',');
    Serial.println(pose.theta);

    int batlev = analogRead(A1);
    float voltage = batlev * (5.0f / 1024.0f) * 3.0f;
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println("V");

    pose_update_millis = this_millis;
  }

  if (this_millis > pid_update_millis + PID_UPDATE_PERIOD) {

    float turn_demand = 0.0f;
    if (state == STATE_RETURN_HOME) {
      float dx = -pose.x;
      float dy = -pose.y;
      float required_angle = atan2f(dy, dx);
      if (required_angle - pose.theta > M_PI) required_angle -= 2 * M_PI;
      if (required_angle - pose.theta < -M_PI) required_angle += 2 * M_PI;
      turn_demand = heading_PID.update(required_angle, pose.theta);
    } else if (state == STATE_FOLLOW_LINE) {
      turn_demand = heading_PID.update(0, calculateM());
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
      confidence -= 0.3f;
      if (confidence < 0.0f) confidence = 0.0f;
    } else {
      lost_line_count = 0;
    }

    pid_update_millis = this_millis;
  }

  // Based on the value of STATE variable,
  // run code for the appropriate robot behaviour.
  if ( state == STATE_DRIVE_FORWARDS ) {
    demand_left = 500.0f;
    demand_right = 500.0f;
    if (crossing_line_break) {
      float dx = pose.x - line_end_x;
      float dy = pose.y - line_end_y;
      if (abs(dx) + abs(dy) > 400) {
        original_theta = pose.theta;
        changeState(STATE_UTURN);
      }
    }
    if (line_left.onLine() || line_centre.onLine() || line_right.onLine()) {
      if (crossing_line_break) {
        line_break_crossed = true;
        crossing_line_break = false;
      }
      changeState(STATE_FOLLOW_LINE);
    }
  } else if ( state == STATE_FOLLOW_LINE ) {
    bool left = line_left.onLine();
    bool centre = line_centre.onLine();
    bool right = line_right.onLine();

    digitalWrite(LED_BUILTIN, (centre) ? HIGH : LOW);
    digitalWrite(30, (left) ? LOW : HIGH);
    digitalWrite(17, (right) ? LOW : HIGH);
    if (!centre && !left && !right && lost_line_count >= LOST_LINE_THRESHOLD) {
      changeState(STATE_CHECK_RIGHT_ANGLE);
      right_angle_state = TURNING_RIGHT;
      original_theta = pose.theta;
    } else {
      float forward_bias = (centre) ? 200.0f : 0.0f;
      float forward_constant = (centre) ? 200.0f : 0.0f;
      demand_left = forward_bias * confidence + forward_constant;
      demand_right = forward_bias * confidence + forward_constant;
    }
  } else if ( state == STATE_CHECK_RIGHT_ANGLE ) {
    // Turn 90 degrees left, then 90 degrees right, then back to centre
    // If the line is seen at any point stop

    if (right_angle_state == TURNING_RIGHT) {
      demand_left = 500.0f;
      demand_right = -500.0f;

      if (abs((wrapAngle(original_theta - M_PI_2)) - pose.theta) < 0.05f) {
        right_angle_state = TURNING_LEFT;
      }
    } else if (right_angle_state == TURNING_LEFT) {
      demand_left = -500.0f;
      demand_right = 500.0f;

      if (abs((wrapAngle(original_theta + M_PI_2)) - pose.theta) < 0.05f) {
        if (line_break_crossed) {
          changeState(STATE_RETURN_HOME);
        } else {
          right_angle_state = TURNING_CENTRE;
        }
      }
    } else if (right_angle_state == TURNING_CENTRE) {
      demand_left = 500.0f;
      demand_right = -500.0f;

      if (abs(original_theta - pose.theta) < 0.05f) {
        changeState(STATE_DRIVE_FORWARDS);
        crossing_line_break = true;
        line_end_x = pose.x;
        line_end_y = pose.y;
      }
    }

    if (line_centre.onLine()) {
      changeState(STATE_FOLLOW_LINE);
    }
  } else if ( state == STATE_UTURN ) {
    demand_left = -500.0f;
    demand_right = 500.0f;

    if (abs((wrapAngle(original_theta - M_PI)) - pose.theta) < 0.05f) {
      crossing_line_break = false;
      changeState(STATE_DRIVE_FORWARDS);
    }
  } else if ( state == STATE_RETURN_HOME ) {
    float dx = -pose.x;
    float dy = -pose.y;
    float dtheta = findAngleDifference(atan2f(dy, dx));
    if (dtheta > 0.2f || dtheta < -0.2f) {
      demand_left = 0.0f;
      demand_right = 0.0f;
    } else {
      if (demand_left == 0.0f) {
        left_PID.reset();
        right_PID.reset();
        demand_left = 500.0f;
        demand_right = 500.0f;
      }
    }

    if (abs(dx) + abs(dy) < 200.0f) {
      stopped = true;
      changeState(STATE_DONE);
    }

  } else if ( state == STATE_DONE ) {
    // Do nothing!
  } else {
    Serial.print("System Error, Unknown state: ");
    Serial.println(state);
  }
  
}
