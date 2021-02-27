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

long prev_left_count, prev_right_count;
unsigned long timestamp;
float current_speed;

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

  prev_left_count = 0;
  prev_right_count = 0;
  current_speed = 0.0f;
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

  unsigned int elapsed_time = micros() - timestamp;

  int left_change = left_count - prev_left_count;
  int right_change = right_count - prev_right_count;

  current_speed = (((left_change + right_change) / 2) / (float)elapsed_time) * 1000000;
  // current_speed *= 0.0001527163f;

  // encoder counts / microseconds
  // * 1000000
  // encoder_counts / seconds
  // 1 count = 0.152716mm = 0.000152716m
  

  prev_left_count = left_count;
  prev_right_count = right_count;

  timestamp = micros();

  Serial.println(current_speed);

  delay(50);

  
} // end of loop()
