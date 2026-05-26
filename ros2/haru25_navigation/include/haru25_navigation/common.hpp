#pragma once
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Core>

#define DEG2RAD(x) ((x) * 0.017453293f)//pi/180
#define RAD2DEG(x) ((x) * 57.295779513f)//180/pi

#define DT 0.005 // 200Hz
#define DT_IMU 0.01 //100Hz
#define DT_PID 0.005  // 200Hz

#define SQRT_1_2 (1.f / M_SQRT2)

typedef struct Pose2D {
    Eigen::Vector2d xy;
    double yaw;
};

typedef struct Point2D {
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
    double vyaw;
};

// 角度を正規化（-pi ~ pi）
inline double normalizeAngle(double theta) {
    return theta - 2 * M_PI * floor((theta + M_PI) / (2 * M_PI));
}

inline double get_time_sec(void) {
    return static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()) / 1000;
}

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define limit_low(amt, low, high) (((amt) > (low) && (amt) < 0.) ? (low) : (((amt) < (high) && amt >= 0.) ? (high) : (amt)))