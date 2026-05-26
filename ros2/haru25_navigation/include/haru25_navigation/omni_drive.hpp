#pragma once

#include <cmath>

class OmniDrive {
    float wheel_base_;      // ロボット中心からタイヤまでの距離
    float wheel_1_radius_;  // 1.f/ タイヤ半径

   public:
    OmniDrive(float wheel_base, float wheel_radius);
        
    // vt→ω(角速度) vx,vy→グローバル座標の速度ベクトル t→yaw角（単位はrad）
    void setRobotSpeed(float vx, float vy, float vt, float t, float *motor_vel);
    void set_wheel_1_radius_(float wheel_radius);

};
