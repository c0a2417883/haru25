#pragma once

#include "Common.h"

class PID{
  float kp_, ki_, kd_;
  float integral_;
  float max_;
  float prev_error_;
  unsigned long prev_ts_;
  public:
  PID(float kp, float  ki, float kd, float  max):
    kp_(kp),
    ki_(ki),
    kd_(kd),
    integral_(0),
    max_(max){}
    
  float update(float error){
    unsigned long ts = micros();//現時刻をマイクロ秒で取得。unsigned longは整数。
    float dt = (ts - prev_ts_) * 1e-6;//マイクロ秒を秒にする。　１Ms = 1*10^-6s
    prev_ts_ = ts;
  
    float out = kp_ * error + kd_ * (error - prev_error_) / dt;//kp_とkd_は比例ゲイン。(error - prev_error_) / dtでerror微分をしてる。
    out = constrain(out, -max_, max_);//outが、-max_とmax_との間にあるときはout、outが-max_より小さいときは-max_、outがmax_より大きいときはmax_を返す
    integral_ += ki_ * error * dt;
    float windup = max_ - fabs(out);//Anti-windup
	  integral_ = (windup > 0) ? constrain(integral_, -max_, max_) : 0.f;
    out += integral_;
    prev_error_ = error;
    return out;
  }
};
