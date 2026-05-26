#pragma once

#include "common.hpp"
#include "omni_drive.hpp"
#include "pid.hpp"
#include "plan.hpp"

class PurePursuit {
    // 経路追従
    Plan *plan_;
    float path_goal_distance_;
    float path_goal_angle_;
    float path_pid_distance_;  // PID制御開始位置
    float max_vel_linear_; //並進最大速度
    float max_vel_angular_;
    float min_vel_linear_;
    float min_vel_angular_;
    PID *pid_yaw_;             // ヨー角制御
    PID *pid_foot_;            // 位置制御
    PID *pid_path_;            // 横方向制御
    int path0_up_=0;
    float path_0_vy_ = 0.2;

   public:
   float path_ff_vx, path_ff_vy, path_ff_vyaw;//FF制御
   float path_fb_v, path_fb_vx, path_fb_vy, path_fb_vyaw;//FB制御
   float path_fb_error;

    PurePursuit(Plan *plan, PID *pid_yaw, PID *pid_foot, PID *pid_path);

    std::vector<Point2D> setPlan(int plan_i, bool path_reverse, bool flip);

    bool pursuit(Pose2D &pos, float &vx, float &vy, float &vt);

    void setParam(float path_goal_distance, 
                    float path_goal_angle, 
                    float path_pid_distance,
                    float max_vel_linear,
                    float max_vel_angular,
                    float min_vel_linear,
                    float min_vel_angular);
};

