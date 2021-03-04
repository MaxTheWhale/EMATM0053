#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <math.h>

#define WHEEL_SEPARATION 975.0f

class kinematics_c {
  public:

    float x;
    float y;
    float theta;

    // Function Prototypes
    kinematics_c();   // constructor 
    void update(int left_change, int right_change);    // update kinematics
    
    
}; // End of class definition.


kinematics_c::kinematics_c() {
  x = 0.0f;
  y = 0.0f;
  theta = 0.0f;
} // end of constructor.

// Routine to execute the update to
// kinematics 
void kinematics_c::update(int left_change, int right_change) {
  int distance = (left_change + right_change) / 2;
  x += distance * cosf(theta);
  y += distance * sinf(theta);
  theta += (right_change - left_change) / WHEEL_SEPARATION;
}


#endif
