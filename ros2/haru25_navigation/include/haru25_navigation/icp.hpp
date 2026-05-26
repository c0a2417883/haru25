#pragma once

#include <vector>
#include "common.hpp"

class ICP {
    double flip_=1;
   public:
    ICP();
    std::vector<Eigen::Vector2d> setScan(std::vector<float> &ranges, Pose2D &pos);
    void setEKF(Point2D &pos);
    void makeMap(bool red);

   private:
    bool eraseMap(double x1, double x2, double y1, double y2);
    bool addWall(double x1, double x2, double y1, double y2, double width);
    double optimizePose(Pose2D &robot_pos, const std::vector<Eigen::Vector2d> &points, const std::vector<int> &wall_ids);
    double calGaussNewton(const Pose2D &pose, const std::vector<Eigen::Vector2d> &points, const std::vector<int> &wall_ids, Pose2D &newPose);
    double robustWeightHuber(double e);
    double robustWeightTukey(double e);
};