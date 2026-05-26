#include "haru25_navigation/omni_drive.hpp"

OmniDrive::OmniDrive(float wheel_base, float wheel_radius)
    : wheel_base_(wheel_base),
      wheel_1_radius_(1.f / wheel_radius) {
}
void OmniDrive::setRobotSpeed(float vx, float vy, float vt, float t, float *motor_vel) {
        // 回転行列
        float c = cosf(-t);
        float s = sinf(-t);
        float vx_local = vx * c - vy * s;
        float vy_local = vx * s + vy * c;

        float vw = wheel_base_ * vt;
        float v1 = vx_local * M_SQRT1_2 - vy_local * M_SQRT1_2 - vw;
        float v2 = vx_local * M_SQRT1_2 + vy_local * M_SQRT1_2 - vw;
        float v3 = -vx_local * M_SQRT1_2 + vy_local * M_SQRT1_2 - vw;
        float v4 = -vx_local * M_SQRT1_2 - vy_local * M_SQRT1_2 - vw;
        motor_vel[0] = v1 * wheel_1_radius_;//角速度を代入
        motor_vel[1] = v2 * wheel_1_radius_;
        motor_vel[2] = v3 * wheel_1_radius_;
        motor_vel[3] = v4 * wheel_1_radius_;
}

void OmniDrive::set_wheel_1_radius_(float wheel_radius){
    wheel_1_radius_ = (1.f / wheel_radius);
}
