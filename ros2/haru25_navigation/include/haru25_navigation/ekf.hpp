#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "common.hpp"
#include <iostream>
using namespace Eigen;
using namespace std;

class EKF {
    // 状態ベクトル [x, y, θ, vx, vy, ω]
    VectorXd xEst_;
    // 共分散行列
    MatrixXd PEst_;
    // 入力ベクトル
    VectorXd U_;
    // 観測ベクトル
    VectorXd Z_;
    MatrixXd P_;        // 共分散行列
    MatrixXd Q_;        // システムノイズ
    MatrixXd R_;        // システムノイズ

    // 状態方程式
    MatrixXd B_;  // システムノイズ
    MatrixXd F_;  // システムノイズ
    MatrixXd H_;  // システムノイズ
    MatrixXd jF_;  // システムノイズ
    MatrixXd jH_;  // システムノイズ
    MatrixXd jH_T_;

    bool observe_ = false;

   public:
    EKF();
    void setOdom(float dx, float dy, float dyaw);
    void setLidar(Pose2D &pos);
    void initPos(float x, float y, float yaw);
    Point2D estimate();

   private:
    void ekf_estimation(VectorXd &xEst, MatrixXd &PEst, VectorXd &z, VectorXd &u);
    VectorXd motion_model(VectorXd &x, VectorXd &u);
    MatrixXd jacob_f(VectorXd &x, VectorXd &u);
    MatrixXd observation_model(VectorXd &x);
};