#ifndef HOMING_H
#define HOMING_H

#include "actuator.h"
#include <string>

class Homing : public Actuator {
public:
    Homing();
    Homing(std::string can_index);
    float *start(int m1, int m2, int m3, float kp, float kd, bool enable_motors = true, bool disable_motors = true);
    float min(float * numberList,int listSize);
    float *arange (float start, float end, float step, int &size);
};

#endif