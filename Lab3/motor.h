#ifndef _MOTOR_H
#define _MOTOR_H

#define DIR_FWD LOW
#define DIR_BWD HIGH

// A class to neatly contain commands for the 
// motors, to take care of +/- values, a min/max
// power value, & pin setup.

class motor_c {
  int pin_dir;
  int pin_pwm;
  
  public:

    // This is a function prototype.
    // The actual function is written outside
    // of the class (see below).
    motor_c( int which_pin_pwm, int which_pin_dir );
    void setPower( int power );
};

// The constructor assigns the pins and sets
// them as outputs.
motor_c::motor_c( int which_pin_pwm, int which_pin_dir ) {
  pin_pwm = which_pin_pwm;
  pin_dir = which_pin_dir;
  pinMode( pin_pwm, OUTPUT );
  pinMode( pin_dir, OUTPUT );
}

void motor_c::setPower( int power ) {

  if ( power > 255 || power < -255 ) {
    Serial.println( "motor_c::setPower: bad power value" );
    return;
  }
  
  int dir = ( power < 0 ) ? DIR_BWD : DIR_FWD;
  if ( power < 0 ) power = -power;
  
  digitalWrite( pin_dir, dir );
  analogWrite( pin_pwm, power );
}

#endif
