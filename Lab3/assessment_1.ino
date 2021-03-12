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
// tabs above.
#include "lineSensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "motor.h"
#include "pid.h"

// You may need to change these depending on how you wire
// in your line sensor.
#define LINE_LEFT_PIN A4   // Pin for the left line sensor
#define LINE_CENTRE_PIN A3 // Pin for the centre line sensor
#define LINE_RIGHT_PIN A2  // Pin for the right line sensor

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define LED_TX 30
#define LED_RX 17

lineSensor_c line_left( LINE_LEFT_PIN );     // Create a line sensor object for the left sensor
lineSensor_c line_centre( LINE_CENTRE_PIN ); // Create a line sensor object for the centre sensor
lineSensor_c line_right( LINE_RIGHT_PIN );   // Create a line sensor object for the right sensor

motor_c left_motor( L_PWM_PIN, L_DIR_PIN );
motor_c right_motor( R_PWM_PIN, R_DIR_PIN );

const float Kp_left = 0.05f;  // Proportional gain 
const float Kd_left = -0.3f;  // Derivative gain
const float Ki_left = 0.003f; // Integral gain
PID_c left_PID( Kp_left, Ki_left, Kd_left ); // controller for left wheel

const float Kp_right = 0.045f; // Proportional gain 
const float Kd_right = -0.3f;  // Derivative gain
const float Ki_right = 0.003f; // Integral gain
PID_c right_PID( Kp_right, Ki_right, Kd_right ); // controller for right wheel

const float Kp_heading = 600.0f; // Proportional gain 
const float Kd_heading = 0.0f;   // Derivative gain
const float Ki_heading = 0.0f;   // Integral gain
PID_c heading_PID( Kp_heading, Ki_heading, Kd_heading ); // controller for heading

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

// Time periods in milliseconds for loop updates
#define PID_UPDATE_PERIOD 100
#define POSE_UPDATE_PERIOD 50

unsigned long pid_update_millis = 0;
unsigned long pose_update_millis = 0;
float demand_left, demand_right;

// Sub-states for right angle check
#define TURNING_RIGHT  0
#define TURNING_LEFT   1
#define TURNING_CENTRE 2

int right_angle_state;
float original_theta;

// Demand value used when driving forward or
// turning on the spot.
#define FORWARD_DEMAND 500.0f

// Thresholds in radians used to check if we
// are 'close enough' to an angle
#define ANGLE_THRESHOLD 0.05f
#define HOME_ANGLE_THRESHOLD 0.2f

// When on the line, forward bias is comprised
// of a constant component and a component that
// varies with confidence.
#define CONFIDENCE_POWER 200.0f
#define CONSTANT_POWER 200.0f

// Sizes in encoder counts
#define HOME_AREA_SIZE 200
#define MAX_LINE_BREAK_SIZE 400

// Rate to increase/decrease confidence when
// on/off the line.
#define CONFIDENCE_INCREASE 0.005f
#define CONFIDENCE_DECREASE 0.3f

// If the total line sensor readings are less than
// this value, m will default to 0.
#define M_MINIMUM_COUNT 300

float confidence;

// Number of updates before the line is considered lost
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
  Serial.begin( 9600 );

  // Delay to connect properly.
  delay( 1000 );

  // Print a debug, so we can see a reset on monitor.
  Serial.println( "***RESET***" );

  // Calibrate the line sensors
  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();

  // Setup the encoders and timer
  setupEncoder1();
  setupEncoder0();
  setupTimer3();

  // Initialise global variables
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

/* Calculate the m value, representing how centred
 * on the line the Romi currently is. Due to occasional
 * negative values from the calibrated line sensors, the
 * value is manually capped to -1 to 1.
 */
float calculateM() {
  int left = line_left.read();
  int centre = line_centre.read();
  int right = line_right.read();
  float total = left + right + centre;
  float m = 0;
  if ( total > M_MINIMUM_COUNT ) {
    float l_norm = left / total;
    float r_norm = right / total;
    m = r_norm - l_norm;
    if ( m > 1.0f ) m = 1.0f;
    if ( m < -1.0f ) m = -1.0f;
  }

  return m;
}

/* Ensure an angle is in the range -PI to PI.
*/
float wrapAngle( float theta ) {
  while ( theta > M_PI || theta < -M_PI ) {
    if ( theta > M_PI ) theta -= 2.0f * M_PI;
    if ( theta < -M_PI ) theta += 2.0f * M_PI;
  }
  return theta;
}

/* Change the state, and also reset the confidence
 * and PID integrals.
 */
void changeState( int new_state ) {
  state = new_state;
  confidence = 0.0f;
  left_PID.reset();
  right_PID.reset();
}

/* Find the change in encoder counts since the last
 * call, and use them to update the kinematics.
 */
void updatePose() {
  int left_change = left_count - prev_loop_left_count;
  int right_change = right_count - prev_loop_right_count;

  prev_loop_left_count = left_count;
  prev_loop_right_count = right_count;

  pose.update( left_change, right_change );
}

/* Read the current voltage of the Romi batteries,
 * as described in the supplementary lab.
 */
float readVoltage() {
  int batlev = analogRead( A1 );
  float voltage = batlev * ( 5.0f / 1024.0f ) * 3.0f;
  return voltage;
}

/* This function uses the heading PID to calculate the
 * turn bias for the motors. When returning home, this
 * is based on the angle difference needed to face home,
 * and when following the line this is based on the m-value.
 */
float getTurnDemand() {
  float turn_demand = 0.0f;
  if ( state == STATE_RETURN_HOME ) {
    float dx = -pose.x;
    float dy = -pose.y;
    float required_angle = atan2f( dy, dx );
    if ( required_angle - pose.theta > M_PI ) required_angle -= 2 * M_PI;
    if ( required_angle - pose.theta < -M_PI ) required_angle += 2 * M_PI;
    turn_demand = heading_PID.update( required_angle, pose.theta );
  } else if ( state == STATE_FOLLOW_LINE ) {
    turn_demand = heading_PID.update( 0, calculateM() );
  }
  return turn_demand;
}

/* Increase the confidence when on the line, decrease the
 * confidence when off of it. Also update the number of
 * updates since the line was lost.
 */
void adjustConfidence() {
  confidence += CONFIDENCE_INCREASE;

  if ( !line_centre.onLine() && !line_left.onLine() && !line_right.onLine() ) {
    lost_line_count += 1;
    confidence -= CONFIDENCE_DECREASE;
    if ( confidence < 0.0f ) confidence = 0.0f;
  } else {
    lost_line_count = 0;
  }
}

// The main loop of execution.  This loop()
// function is automatically called every
// time it finishes.  You should try to write
// your code to take advantage of this looping
// behaviour.
void loop() {

  // Store the millis value so we don't keep
  // calling it for every check.
  unsigned long this_millis = millis();

  /* The POSE_UPDATE handles updating the kinematics and also
     prints the pose and battery voltage to serial.
   */
  if ( this_millis > pose_update_millis + POSE_UPDATE_PERIOD ) {

    updatePose();
    Serial.print( "Pose: " );
    Serial.print( pose.x );
    Serial.print( ',' );
    Serial.print( pose.y );
    Serial.print( ',' );
    Serial.println( pose.theta );

    float voltage = readVoltage();
    Serial.print( "Voltage: " );
    Serial.print( voltage );
    Serial.println( 'V' );

    pose_update_millis = this_millis;
  }

  /* The PID_UPDATE handles updating the PIDs for the motors,
   * and also adjusts the confidence and outputs the current
   * line sensor states to the LEDs
   */
  if ( this_millis > pid_update_millis + PID_UPDATE_PERIOD ) {

    if ( !stopped ) {
      float turn_demand = getTurnDemand();
      float output_left = left_PID.update( demand_left - turn_demand, left_speed );
      float output_right = right_PID.update( demand_right + turn_demand, right_speed );

      left_motor.setPower( output_left );
      right_motor.setPower( output_right );
    } else {
      left_motor.setPower( 0 );
      right_motor.setPower( 0 );
    }

    adjustConfidence();

    // Show the line sensor states on the three LEDs
    digitalWrite( LED_BUILTIN, ( line_centre.onLine() ) ? HIGH : LOW );
    digitalWrite( LED_TX, ( line_left.onLine() ) ? LOW : HIGH );
    digitalWrite( LED_RX, ( line_right.onLine() ) ? LOW : HIGH );

    pid_update_millis = this_millis;
  }

  // Based on the value of STATE variable,
  // run code for the appropriate robot behaviour.

  /* The DRIVE_FORWARDS state simply command the Romi to
   * drive straight, until any of the line sensors detect
   * a line. It also has a special check if the Romi is 
   * currently trying to cross a line break, to switch to
   * a u-turn if it can't find the line again.
   */
  if ( state == STATE_DRIVE_FORWARDS ) {
    demand_left = FORWARD_DEMAND;
    demand_right = FORWARD_DEMAND;
    if ( crossing_line_break ) {
      float dx = pose.x - line_end_x;
      float dy = pose.y - line_end_y;
      if ( abs(dx) + abs(dy) > MAX_LINE_BREAK_SIZE ) {
        original_theta = pose.theta;
        changeState( STATE_UTURN );
      }
    }
    if ( line_left.onLine() || line_centre.onLine() || line_right.onLine() ) {
      if ( crossing_line_break ) {
        line_break_crossed = true;
        crossing_line_break = false;
      }
      changeState( STATE_FOLLOW_LINE );
    }
  /* The FOLLOW_LINE state follows the line using the heading
   * PID and the m-value, with an increasing confidence if it
   * stays on the line. If it loses the line for more than
   * LOST_LINE_THRESHOLD consecutive updates, it switches to
   * the right angle checking behaviour.
   */
  } else if ( state == STATE_FOLLOW_LINE ) {
    bool left = line_left.onLine();
    bool centre = line_centre.onLine();
    bool right = line_right.onLine();

    if ( !centre && !left && !right && lost_line_count >= LOST_LINE_THRESHOLD ) {
      changeState( STATE_CHECK_RIGHT_ANGLE );
      right_angle_state = TURNING_RIGHT;
      original_theta = pose.theta;
    } else {
      demand_left = CONFIDENCE_POWER * confidence + CONSTANT_POWER;
      demand_right = CONFIDENCE_POWER * confidence + CONSTANT_POWER;
    }
  /* The CHECK_RIGHT_ANGLE state first turns 90 degrees to the right,
   * then 90 degrees to the left. If it finds the line at any point,
   * it switches back to the line follow state. If it doesn't find the
   * line, it will either attempt to cross the line break, or switch to
   * returning home if this has already been done.
   */
  } else if ( state == STATE_CHECK_RIGHT_ANGLE ) {

    if ( right_angle_state == TURNING_RIGHT ) {
      demand_left = FORWARD_DEMAND;
      demand_right = -FORWARD_DEMAND;

      if ( abs( ( wrapAngle( original_theta - M_PI_2 ) ) - pose.theta ) < ANGLE_THRESHOLD ) {
        right_angle_state = TURNING_LEFT;
      }
    } else if ( right_angle_state == TURNING_LEFT ) {
      demand_left = -FORWARD_DEMAND;
      demand_right = FORWARD_DEMAND;

      if ( abs( ( wrapAngle( original_theta + M_PI_2 ) ) - pose.theta ) < ANGLE_THRESHOLD ) {
        if ( line_break_crossed ) {
          changeState( STATE_RETURN_HOME );
        } else {
          right_angle_state = TURNING_CENTRE;
        }
      }
    } else if ( right_angle_state == TURNING_CENTRE ) {
      demand_left = FORWARD_DEMAND;
      demand_right = -FORWARD_DEMAND;

      if ( abs( original_theta - pose.theta ) < ANGLE_THRESHOLD ) {
        changeState( STATE_DRIVE_FORWARDS );
        crossing_line_break = true;
        line_end_x = pose.x;
        line_end_y = pose.y;
      }
    }

    if ( line_centre.onLine() ) {
      changeState( STATE_FOLLOW_LINE );
    }
  /* The UTURN state is fairly simple, it simply turns the Romi left
   * until it has reached 180 degrees away from it's previous angle,
   * then switches to driving forwards.
   */
  } else if ( state == STATE_UTURN ) {
    demand_left = -FORWARD_DEMAND;
    demand_right = FORWARD_DEMAND;

    if ( abs( ( wrapAngle( original_theta - M_PI ) ) - pose.theta ) < ANGLE_THRESHOLD ) {
      crossing_line_break = false;
      changeState( STATE_DRIVE_FORWARDS );
    }
  /* The RETURN_HOME state first uses the heading PID in combination
   * with the required angle to face (0, 0) to turn towards home. Once
   * it is close enough to the right direction, it begins driving
   * forwards, but the heading controller will still add a turning bias
   * to correct its course. Once it's very close, it stops.
   */
  } else if ( state == STATE_RETURN_HOME ) {
    float dx = -pose.x;
    float dy = -pose.y;
    float dtheta = wrapAngle( atan2f( dy, dx ) - pose.theta );
    if ( abs( dtheta ) > HOME_ANGLE_THRESHOLD ) {
      demand_left = 0.0f;
      demand_right = 0.0f;
    } else {
      if ( demand_left == 0.0f ) {
        left_PID.reset();
        right_PID.reset();
        demand_left = FORWARD_DEMAND;
        demand_right = FORWARD_DEMAND;
      }
    }

    if ( abs( dx ) + abs( dy ) < HOME_AREA_SIZE ) {
      stopped = true;
      changeState( STATE_DONE );
    }

  } else if ( state == STATE_DONE ) {
    // Do nothing!
  } else {
    Serial.print( "System Error, Unknown state: " );
    Serial.println( state );
  }
  
}
