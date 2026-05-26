#include "haru25_navigation/ekf.hpp"

EKF::EKF() : xEst_(6),
                                                     PEst_(6, 6),
                                                     U_(3),
                                                     Z_(3),
                                                     P_(6, 6),
                                                     Q_(6, 6),
                                                     R_(6, 6),
                                                     B_(6, 3),
                                                     F_(6, 6),
                                                     H_(3, 6),
                                                     jF_(6, 6),
                                                     jH_(3, 6) {
    //  Covariance for EKF simulation
    xEst_ = VectorXd::Zero(6);
    PEst_ = MatrixXd::Identity(6, 6);
    U_ = VectorXd::Zero(3);
    Z_ = VectorXd::Zero(3);
    VectorXd v6(6);
    v6 << 0.001, 0.001, DEG2RAD(1.0), 0.001, 0.001, DEG2RAD(1.0);
    v6 = v6.array().square();
    Q_ = v6.asDiagonal();
    VectorXd v3(3);
    v3 << 0.01, 0.01, 0.01;
    v3 = v3.array().square();
    R_ = v3.asDiagonal();
    B_ = MatrixXd::Zero(6, 3);
    B_(2, 2) = DT;
    B_(3, 0) = 1;
    B_(4, 1) = 1;
    B_(5, 2) = 1;
    F_ = MatrixXd::Zero(6, 6);
    F_(0, 0) = 1;
    F_(1, 1) = 1;
    F_(2, 2) = 1;
    H_ = MatrixXd::Zero(3, 6);
    H_(0, 0) = 1;
    H_(1, 1) = 1;
    H_(2, 2) = 1;
    jF_ = MatrixXd::Identity(6, 6);
    jF_(2, 5) = DT;
    jH_ = MatrixXd::Zero(3, 6);
    jH_(0, 0) = 1;
    jH_(1, 1) = 1;
    jH_(2, 2) = 1;
    jH_T_ = jH_.transpose();
}

void EKF::initPos(float x, float y, float yaw) {
    cout << PEst_ << endl;
    xEst_ << x, y, yaw, 0, 0, 0;
    U_ << 0, 0, 0;
    PEst_ << 3.78115e-05,-1.1578e-10,-1.19088e-10,3.20037e-06,2.94232e-05,-2.49235e-10,
        -1.1578e-10,3.78133e-05,8.89931e-10,-2.94289e-05,3.20026e-06,5.27258e-09,
        -1.19088e-10,8.89931e-10 ,0.00104439 ,-2.42351e-10,3.79718e-11,0.00102298,
        3.20037e-06,- 2.94289e-05,-2.42351e-10,0.000126724,-7.77294e-10,-1.58255e-08,
        2.94232e-05,3.20026e-06,3.79718e-11,-7.77294e-10,0.000126706,7.50693e-10,
        -2.49235e-10,5.27258e-09,0.00102298,-1.58255e-08,7.50693e-10,0.0316329;
}

void EKF::setOdom(float dx, float dy, float dyaw){
    U_(0) = dx / DT;
    U_(1) = dy / DT;
    U_(2) = dyaw / DT_IMU;
}
void EKF::setLidar(Pose2D &pos){
    Z_(0) = pos.xy(0);
    Z_(1) = pos.xy(1);
    Z_(2) = pos.yaw;
    observe_ = true;
}

Point2D EKF::estimate(){
    ekf_estimation(xEst_, PEst_, Z_, U_);
    Point2D point;
    point.x     = xEst_(0);
    point.y     = xEst_(1);
    point.yaw = normalizeAngle(xEst_(2));
    point.vx    = xEst_(3);
    point.vy    = xEst_(4);
    point.vyaw  = xEst_(5);
    return point;
}

void EKF::ekf_estimation(VectorXd &xEst, MatrixXd &PEst, VectorXd &z, VectorXd &u) {
    // Predict
    VectorXd xPred = motion_model(xEst, u);
    MatrixXd jF = jacob_f(xEst, u);
    MatrixXd PPred = jF * PEst * jF.transpose() + Q_;
    if(!observe_){
        xEst = xPred;
        PEst = PPred;
    }else{
        // Update
        MatrixXd zPred = observation_model(xPred);
        VectorXd y = z - zPred;
        y(2) = normalizeAngle(y(2));
        MatrixXd S = jH_ * PPred * jH_T_ + R_;
        MatrixXd K = PPred * jH_T_ * S.inverse();
        xEst = xPred + K * y;
        PEst = (MatrixXd::Identity(6, 6) - K * jH_) * PPred;
        observe_ = false;
    }
}

VectorXd EKF::motion_model(VectorXd &x, VectorXd &u) {
    double yaw = x(2);
    double c = cos(yaw) * DT;
    double s = sin(yaw) * DT;
    B_(0, 0) = c;
    B_(0, 1) = -s;
    B_(1, 0) = s;
    B_(1, 1) = c;
    return F_ * x + B_ * u;
}

MatrixXd EKF::jacob_f(VectorXd &x, VectorXd &u) {
    double vx, vy, vt, yaw, c, s;
    vx = u(0, 0);
    vy = u(1, 0);
    vt = u(2, 0);
    yaw = x(2, 0);
    c = cos(yaw) * DT;
    s = sin(yaw) * DT;
    jF_(0, 2) = -vx * s - vy * c;
    jF_(0, 3) =  c;
    jF_(0, 4) = -s;
    jF_(1, 2) = vx * c - vy *  s;
    jF_(1, 3) =  s;
    jF_(1, 4) =  c;
    return jF_;
}

MatrixXd EKF::observation_model(VectorXd &x) {
    return H_ * x;
}