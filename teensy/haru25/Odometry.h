#pragma once
#include "Imu.h"
#include "Encoder.h"
#include "Common.h"

#define ENCODER_PPR (2048)//defineは値に()をつける　分解能2048
#define ENCODER_POS_PER_PULSE (M_PI * 2.0 / ENCODER_PPR / 4)//M_PI * 2.0 = 2π

class Odometry{
  Encoder *encoder_;
  Imu *imu_;

  float cnt2meter1_;
  float cnt2meter2_;

  double x_;
  double y_;
  int pre_cnt1_;
  int pre_cnt2_;
  
  public:
  // コンストラクタ
  Odometry(Encoder *encoder, Imu *im, float wheel_radius1, float wheel_radius2)
  :encoder_(encoder),
   imu_(im),
   cnt2meter1_(ENCODER_POS_PER_PULSE * wheel_radius1),
   cnt2meter2_(ENCODER_POS_PER_PULSE * wheel_radius2)
  {
  }
  void init(){
    x_=0;
    y_=0;
    pre_cnt1_=0;
    pre_cnt2_=0; 
    
  }
  Pos get(){
    Pos p;
    p.t = imu_->getYaw();
    int cnt1, cnt2;
    encoder_->getCounts(cnt1, cnt2);
    float diff1, diff2;
    diff1 = (float)(cnt1 - pre_cnt1_);
    diff2 = (float)(cnt2 - pre_cnt2_);
    pre_cnt1_ = cnt1;
    pre_cnt2_ = cnt2;
    float dx= diff1 * cnt2meter1_;
    float dy= diff2 * cnt2meter2_;
    float c = cosf(p.t);
    float s = sinf(p.t);
    float dx_global = dx*c-dy*s;
    float dy_global = dx*s+dy*c;
    x_ += dx_global;
    y_ += dy_global;  
    p.x = x_;
    p.y = y_;
    return p;
  }
};
