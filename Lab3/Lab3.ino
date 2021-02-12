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

} // end of setup()

void BangBang() {
  int left = line_left.read();
  int centre = line_centre.read();
  int right = line_right.read();
  float total = left + right + centre;
  float m = 0;
  if (total > 100) {
    float l_norm = left / total;
    float c_norm = centre / total;
    float r_norm = right / total;
    m = l_norm - r_norm;
  }
  Serial.println(m);
  bool left_black = (left > 300);
  bool centre_black = (centre > 300);
  bool right_black = (right > 300);

  int l_power = (int)(20.0f * m);
  int r_power = (int)(-20.0f * m);

  if (left_black && !right_black) {
    l_power = -20;
    r_power = 20;
  } else if (right_black && !left_black) {
    l_power = 20;
    r_power = -20;
  } else if (centre_black) {
    l_power = 20;
    r_power = 20;    
  }
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

  BangBang();

  delay(50);
  
} // end of loop()
