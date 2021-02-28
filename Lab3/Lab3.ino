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

long prev_loop_left_count, prev_loop_right_count;
unsigned long timestamp;
float loop_left_speed, loop_right_speed;

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

} // end of setup()

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
    m = l_norm - r_norm;
    if (m < -1.0f || m > 1.0f) Serial.println("calculate_m: bad m value");
  }

  return m;
}

#define PID_UPDATE_PERIOD 100
#define SPEED_UPDATE_PERIOD 50

unsigned long switch_millis = 0;
unsigned long pid_update_millis = 0;
unsigned long speed_update_millis = 0;
float demand = 1000.0f;

// The main loop of execution.  This loop()
// function is automatically called every
// time it finishes.  You should try to write
// your code to take advantage of this looping
// behaviour.  
void loop() {

  // To send data back to your computer.
  // You can open either Serial monitor or plotter.
//  Serial.print( line_left.read() );
//  Serial.print( ", " );
//  Serial.print( line_centre.read() );
//  Serial.print( ", " );
//  Serial.print( line_right.read() );
//  Serial.print( "\n" );

  // Serial.print(left_count);
  // Serial.print(", ");
  // Serial.print(right_count);
  // Serial.print("\n");

//  delay(50);
  // if (left_count < 2000) left_motor.setPower(50);
  // else left_motor.setPower(0);
  // if (right_count < 2000) right_motor.setPower(50);
  // else right_motor.setPower(0);

  

  //Serial.print("From loop: ");

  unsigned long this_millis = millis();

  if (this_millis > speed_update_millis + SPEED_UPDATE_PERIOD) {
    unsigned int elapsed_time = micros() - timestamp;

    int left_change = left_count - prev_loop_left_count;
    int right_change = right_count - prev_loop_right_count;

    loop_left_speed = left_change / (float)elapsed_time * 1000000;
    loop_right_speed = right_change / (float)elapsed_time * 1000000;
    // current_speed *= 0.0001527163f;

    // encoder counts / microseconds
    // * 1000000
    // encoder_counts / seconds
    // 1 count = 0.152716mm = 0.000152716m
    

    prev_loop_left_count = left_count;
    prev_loop_right_count = right_count;

    timestamp = micros();
  }

  if (this_millis > switch_millis + 4000) {
    demand = -demand;
    switch_millis = this_millis;
  }

  if (this_millis > pid_update_millis + PID_UPDATE_PERIOD) {
    //    output_signal <----PID-- demand, measurement
    float turn_demand = heading_PID.update(0, calculate_m());
    float output_left = left_PID.update(200 + turn_demand, left_speed);
    float output_right = right_PID.update(200 - turn_demand, right_speed);

    // Serial.print(turn_demand);
    // Serial.print(",");
    // Serial.print(output_left);
    // Serial.print(",");
    // Serial.println(output_right);

    left_motor.setPower(output_left);
    right_motor.setPower(output_right);

    pid_update_millis = this_millis;

    Serial.println(calculate_m());
  }
  
} // end of loop()
