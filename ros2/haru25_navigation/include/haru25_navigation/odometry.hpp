#pragma once
#include "common.hpp"

#define ENCODER_PPR (2048)//defineは値に()をつける　分解能2048
#define ENCODER_POS_PER_PULSE (M_PI * 2.0 / ENCODER_PPR / 4)//(2π/分解能)/4 カウント1のときの角度

class Odometry{

  double cnt2meter1_;
  double cnt2meter2_;

  double x_;
  double y_;
  double yaw_;

  public:
  // コンストラクタ
  Odometry(float wheel_radius1, float wheel_radius2);//オドメーターのタイヤの半径
  void init(float x, float y, float yaw);
  Pose2D getPos(float local_dx, float local_dy, float dyaw);
  Pose2D getLocal(float local_dx, float local_dy);
  Pose2D getLocal2(float *wheel_v);
  void getVel(float dx, float dy, float dyaw);
  float vx;
  float vy;
  float vyaw;
};
