#pragma once

#include "Common.h"
#include "OmniDrive.h"
#include "PID.h"
#include "Plan.h"

class PurePursuit {
    // 経路追従
    Plan *plan_;
    float path_aim_distance2_;
    float path_goal_distance_;
    float path_goal_angle_;
    float path_pid_distance_;  // PID制御開始位置
    float velocity_limit_linear_; //並進最大速度
    PID *pid_yaw_;             // ヨー角制御
    PID *pid_foot_;            // 位置制御
    // 足回り
    OmniDrive *omni_;

   public:
    PurePursuit(Plan *plan, OmniDrive *omni, PID *pid_yaw, PID *pid_foot, float path_aim_distance, float path_goal_distance, float path_goal_angle, float path_pid_distance, float velocity_limit_linear) : plan_(plan),
                                                                                                                                                                               omni_(omni),
                                                                                                                                                                               pid_yaw_(pid_yaw),
                                                                                                                                                                               pid_foot_(pid_foot),
                                                                                                                                                                               path_aim_distance2_(path_aim_distance * path_aim_distance),
                                                                                                                                                                               path_goal_distance_(path_goal_distance),
                                                                                                                                                                               path_goal_angle_(path_goal_angle),
                                                                                                                                                                               path_pid_distance_(path_pid_distance) ,
                                                                                                                                                                               velocity_limit_linear_(velocity_limit_linear){
    }

    void setPlan(int plan_i, bool path_reverse) {
        plan_->setPlan(plan_i, path_reverse);
    }

    bool pursuit(Pos &pos) {
        // 最終地点までの距離, 最終地点の角度
        float last_distance, last_angle;
        plan_->getGoal(pos, last_distance, last_angle);
        
        // 角度の偏差
        float err_angle = normalizeAngle(last_angle - pos.t);
        if (last_distance < path_goal_distance_ && fabs(err_angle) < path_goal_angle_) {
            Serial.println("Goal");
            omni_->setRobotSpeed(0, 0, 0, pos.t);  // 停止
            return true;
        }

        // ロボットと近い経路点を探索
        Pos near_pos;
        float near_vel;
        plan_->getNearPoint(pos, near_pos, near_vel);

        // 少し前の経路点を取得
        Pos aim_pos;
        plan_->getRangePoint(pos, aim_pos, path_aim_distance2_);

        // ロボットが経路点まで動く処理
        // 単位ベクトル（長さが１）を求める
        float aim_dx = aim_pos.x - pos.x;
        float aim_dy = aim_pos.y - pos.y;
        float aim_distance_1 = 1.0 / hypotf(aim_dx, aim_dy);
        aim_dx *= aim_distance_1;
        aim_dy *= aim_distance_1;

        float v;                                   // 速さ
        if (last_distance > path_pid_distance_) {  // ゴールから離れてる
            v = near_vel;                          // 速さを早くする
            err_angle = normalizeAngle(near_pos.t - pos.t);// 途中は近い経路点の角度を参照
        } else                                     // ゴールに近い
            v = pid_foot_->update(last_distance);  // 速さを遅くする（ヒョイッと動く）

        // 並進速度制限
        v = constrain(v, -velocity_limit_linear_, velocity_limit_linear_);

        float vx, vy, vt;
        vx = v * aim_dx;  // ベクトルの大きさがｖの目標点までの向きのベクトルを計算
        vy = v * aim_dy;
        // ロボット全体の角速度(vt)を求める
        vt = pid_yaw_->update(err_angle);  // ロボットの角度をaim_tになるようにPD制御
        omni_->setRobotSpeed(vx, vy, vt, pos.t);
        //Serial.println(vx);
        //Serial.println(vy);
        //Serial.println(vt);

        return false;
    }
};

