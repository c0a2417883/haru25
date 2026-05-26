#pragma once

#include <cmath>
#include "Motor2.h"

#define SQRT_1_2 (1.f / M_SQRT2)

class OmniDrive {
    float wheel_base_;      // ロボット中心からタイヤまでの距離
    float wheel_1_radius_;  // 1.f/ タイヤ半径の逆数
    Motor2 *m12_, *m34_;

   public:
    OmniDrive(float wheel_base, float wheel_radius, Motor2 *m12, Motor2 *m34)
        : wheel_base_(wheel_base),
          wheel_1_radius_(1.f / wheel_radius),
          m12_(m12),
          m34_(m34)
        {
        }
    // vt→ω(角速度) vx,vy→グローバル座標の速度ベクトル t→yaw角（単位はrad）
    void setRobotSpeed(float vx, float vy, float vt, float t) {
        // 回転行列
        float c = cosf(-t);
        float s = sinf(-t);
        float vx_local = vx * c - vy * s;
        float vy_local = vx * s + vy * c;
        

        float vw = wheel_base_ * vt;
        float v1 = vx_local * SQRT_1_2 - vy_local * SQRT_1_2 - vw;
        float v2 = vx_local * SQRT_1_2 + vy_local * SQRT_1_2 - vw;
        float v3 = -vx_local * SQRT_1_2 + vy_local * SQRT_1_2 - vw;
        float v4 = -vx_local * SQRT_1_2 - vy_local * SQRT_1_2 - vw;
        m12_->setSpeed(-v2 * wheel_1_radius_, -v3 * wheel_1_radius_);
        m34_->setSpeed(-v4 * wheel_1_radius_, -v1 * wheel_1_radius_);
    }
};
