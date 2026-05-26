#pragma once

#include "common.hpp"

class Plan {
    // 経路計画データ
    int plan_size_;
    // 経路追従
    int path_num_;
    bool path_reverse_;
    int path_near_i_;
    int path_rage_i_;
    int plan_i_;
    float flip_;

    std::vector<std::vector<Point2D>> plan_;

   public:
    Plan();
    std::vector<Point2D> setPlan(int plan_i, bool path_reverse, bool flip);
    int readPlan(std::string &path);
    // 最近某探索
    bool getNearPoint(Pose2D &robot_pos, Point2D &p1, Point2D &p2);

    double getGoal(Pose2D &pos, Point2D &goal);

   private:
    int getPathPoint(int path_i, Point2D &p);
    int getPathIndex(int path_i);
    double getDistanceSquare(Pose2D &p1, Point2D &p2);
};
