#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#ifndef LINE_THRESHOLD
#define LINE_THRESHOLD 100
#endif

class lineSensor_c {
  
  public:

    int pin;
    int bias = 0;

    // Constructor, accepts a pin number as
    // argument and sets this as input.
    lineSensor_c( int which_pin ) {

       // Record which pin we used.
       pin = which_pin;

       // Set this pin to input.
       pinMode( pin, INPUT );
    };


    // Write your calibration routine here
    // to remove bias offset
    void calibrate() {
      float value = 0;

      // To help get you started, we use 
      // the pin set/recorded by the 
      // constructor to do an analogRead.
      for (int i = 0; i < 100; i++) {
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


    // Write a routine here to check if your
    // sensor is on a line (true or false).
    boolean onLine() {

      if (this->read() > LINE_THRESHOLD) {
          return true;
      }
             
      return false;
    }

    // You can define other functions for
    // yourself. 
    // ...

};

#endif
