#pragma once

#include "common.hpp"

class PID {
    float kp_, ki_, kd_;
    float integral_;
    float max_;
    float prev_error_;

   public:
    PID(float kp, float ki, float kd, float max);
    float update(float error);
    void setGain(float kp, float ki, float kd, float max);
};
