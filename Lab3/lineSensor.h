#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#ifndef LINE_THRESHOLD
#define LINE_THRESHOLD 300
#endif

class lineSensor_c {
  
  public:

    int pin;
    int bias;

    // Constructor, accepts a pin number as
    // argument and sets this as input.
    lineSensor_c( int which_pin ) {

      // Record which pin we used.
      pin = which_pin;

      bias = 0;

      // Set this pin to input.
      pinMode( pin, INPUT );
    };


    // This calibrates the line sensor so that
    // the material currently under it is at 0.
    void calibrate() {
      float value = 0;

      // We do 100 readings and take an average.
      for ( int i = 0; i < 100; i++ ) {
        value += analogRead( pin ); 
      }
      
      value /= 100;
      bias = value;
    }

    int read() {
      return analogRead( pin ) - bias;
    }

    int readRaw() {
      return analogRead( pin );
    }


    // Checks if the sensor is currently
    // on a line.
    boolean onLine() {

      if ( this->read() > LINE_THRESHOLD ) {
          return true;
      }
             
      return false;
    }

};

#endif
