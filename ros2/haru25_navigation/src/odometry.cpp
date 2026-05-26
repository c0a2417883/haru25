#include "haru25_navigation/odometry.hpp"

Odometry::Odometry(float wheel_radius1, float wheel_radius2)
:cnt2meter1_(ENCODER_POS_PER_PULSE * wheel_radius1),
  cnt2meter2_(ENCODER_POS_PER_PULSE * wheel_radius2)
{
  init(0,0,0);
}
void Odometry::init(float x, float y, float yaw){
  x_=x;
  y_=y;
  yaw_=yaw;//機体をもちあげたときのyawのずれ
  
}
Pose2D Odometry::getPos(float local_dx, float local_dy, float dyaw){
Pose2D p;
  yaw_ += dyaw;
  p.yaw = normalizeAngle(yaw_);
  double dx= local_dx * cnt2meter1_;
  double dy= local_dy * cnt2meter2_;
  double c = cos(p.yaw);
  double s = sin(p.yaw);
  double dx_global = dx*c-dy*s;
  double dy_global = dx*s+dy*c;
  x_ += dx_global;
  y_ += dy_global;  
  p.xy[0] = x_;
  p.xy[1] = y_;
  p.yaw = yaw_;
  return p;
}
Pose2D Odometry::getLocal(float local_dx, float local_dy){
  Pose2D p;
  p.xy(0) = local_dx * cnt2meter1_;
  p.xy(1) = local_dy * cnt2meter2_;
  return p;
}
Pose2D Odometry::getLocal2(float *wheel_v){
    Pose2D p;
    p.xy(0) = -0.05 * (wheel_v[3] - wheel_v[2]) * M_SQRT1_2 * DT;
    p.xy(1) = -0.05 * (-wheel_v[1] + wheel_v[3]) * M_SQRT1_2 * DT;
    return p;
}
void Odometry::getVel(float dx, float dy, float dyaw){
  vyaw = dyaw / 0.01;
  vx = dx / 0.01;
  vy = dy / 0.01;
}