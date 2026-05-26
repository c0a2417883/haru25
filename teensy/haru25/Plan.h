#pragma once
#include "Common.h"

class Plan {
    // 経路計画データ
    int plan_size_;
    // 経路追従
    int path_num_;
    bool path_reverse_;
    int path_near_i_;
    int path_rage_i_;

    float *path_x_;
    float *path_y_;
    float *path_t_;
    float *path_v_;

   public:
    Plan() {
        plan_size_ = sizeof(_plan_size) / sizeof(int);
    }
    bool setPlan(int plan_i, bool path_reverse) {
        if (plan_i >= plan_size_)
            return false;
        path_reverse_ = path_reverse;
        // ポインタを取り出す
        path_x_ = _plan[plan_i][0];
        path_y_ = _plan[plan_i][1];
        path_t_ = _plan[plan_i][2];
        path_v_ = _plan[plan_i][3];
        // 経路点の数
        path_num_ = _plan_size[plan_i];
        Serial.print("\nSet Path: ");
        Serial.print(plan_i);
        Serial.print(" : ");
        Serial.println(path_num_);
        path_near_i_ = 0;
        path_rage_i_ = 0;
        return true;
    }
    // 最近某探索
    void getNearPoint(Pos &robot_pos, Pos &p, float &vel) {
        float d, prev_d;
        int i = path_near_i_;
        // まずはiを増やす方針
        getPathPoint(i, p);
        prev_d = getDistanceSquare(robot_pos, p);
        for (; i < path_num_; ++i) {
            getPathPoint(i + 1, p);
            d = getDistanceSquare(robot_pos, p);

            // 一個進めた方が距離が大きくなったらやめる
            if (prev_d < d)
                break;
            prev_d = d;
        }
        getPathPointYawVel(i, p, vel);
        path_near_i_ = i;
    }
    void getRangePoint(Pos &robot_pos, Pos &p, float &range2) {
        int i = path_rage_i_;
        float d;
        getPathPoint(i, p);
        for (; i < path_num_; ++i) {
            getPathPoint(i, p);
            d = getDistanceSquare(robot_pos, p);
            if (d > range2)
                break;
        }
        path_rage_i_ = i;
    }

    void getGoal(Pos &pos, float &distance, float &angle) {
        Pos goal_pos;
        int i = getPathPoint(path_num_ - 1, goal_pos);
        distance = sqrtf(getDistanceSquare(pos, goal_pos));
        angle = path_t_[i];
    }

   private:
    int getPathPoint(int path_i, Pos &pos) {
        int i = getPathIndex(path_i);
        pos.x = path_x_[i];
        pos.y = path_y_[i];
        return i;
    }
    int getPathPointYawVel(int path_i, Pos &pos, float &vel) {
        int i = getPathPoint(path_i, pos);
        pos.t = path_t_[i];
        vel = path_v_[i];
        return i;
    }
    int getPathIndex(int path_i) {
        if (path_i >= path_num_)
            path_i = (path_num_ - 1);
        else if (path_i < 0)
            path_i = 0;

        int i;
        if (path_reverse_)
            i = path_num_ - path_i - 1;
        else
            i = path_i;
        return i;
    }
    float getDistanceSquare(Pos &pos1, Pos &pos2) {
        float dx = pos1.x - pos2.x;
        float dy = pos1.y - pos2.y;
        return dx * dx + dy * dy;
    }
};
