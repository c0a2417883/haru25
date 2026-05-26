#include "haru25_navigation/pid.hpp"

PID::PID(float kp, float ki, float kd, float max) : kp_(kp),
                                                        ki_(ki),
                                                        kd_(kd),
                                                        integral_(0),
                                                        max_(max){

}

float PID::update(float error) {
    float output, potential, differential, integral_diff;
    // P制御
    potential = kp_ * error;
    // 台形積分
    integral_diff = ki_ * 0.5f * (error + prev_error_) * DT_PID;

    differential = kd_ * (error - prev_error_) / DT_PID;

    prev_error_ = error;

    // P制御 + I制御 + D制御
    output = potential + integral_ + integral_diff + differential;
    // antiwindup - limit the output
    if (output > max_) {
        if (integral_diff < 0) {
            integral_ += integral_diff;
        }
        return max_;
    } else if (output < -max_) {
        if (integral_diff > 0) {
            integral_ += integral_diff;
        }
        return -max_;
    }
    integral_ += integral_diff;
    return output;
}

void PID::setGain(float kp, float ki, float kd, float max) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_ = max;
    integral_ = 0;
}