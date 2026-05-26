#include "haru25_navigation/icp.hpp"

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <set>
#include <vector>

#define YAW_INTERPOL
// #define VIEW_MAP
#define EKF_BUFF 30
#define ROBUST

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define GRID_RANGE (10.f)  // グリッド　10m×10m

#define GRID_UNIT (0.025f)                           // グリッドの間隔 (m)
#define GRID_SIZE (int)(GRID_RANGE / GRID_UNIT * 2)  // 配列の大きさ
#define GRID_MID (GRID_SIZE / 2)
#define GRID_GET_N(x) (static_cast<int>(((x) / (GRID_UNIT)) + (GRID_MID)))
#define GRID_GET_INDEX(x, y) (GRID_GET_N(x) + GRID_SIZE * GRID_GET_N(y))
#define GRID_OUT(x) ((x) >= GRID_SIZE * GRID_SIZE)
#define GRID_GET_X(i) ((double)(i - GRID_MID)*GRID_UNIT)

typedef struct Wall {
    Eigen::Vector2d n;  // 法線ベクトル
    Eigen::Matrix2d W;
    Eigen::Vector2d p;  // 壁上の点
};

typedef struct Voxel {
    Eigen::Vector2d p{0, 0};  // 重心
    int len = 0;              // 代入回数
};

typedef struct Transform {
    double time_stamp;        // timestamp
    Eigen::Vector2d xy{0, 0};  // 並行
    Eigen::Vector2d v{0, 0};
#ifndef YAW_INTERPOL
    Eigen::Rotation2Dd R;
#endif
    double yaw, vyaw;
};

// param
int angle_min = 70;
int angle_max = 430;
// 状態変数
std::vector<Eigen::Vector2d> cosSin;
std::vector<Wall> walls;
std::array<uint8_t, GRID_SIZE * GRID_SIZE> wall_grid;

Transform pos_ekf[EKF_BUFF];
int pos_ekf_i=0;

Pose2D lidar_pos;  // LiDARの相対位置
Pose2D pos_odom_conv;
double robot_edge_2_;
double evlimit = 0.05;

ICP::ICP() {
    makeMap(true);
    // LiDARの位置
    lidar_pos.xy = Eigen::Vector2d(-0.206902, -0.207186);
    lidar_pos.yaw = M_PI_4;
    robot_edge_2_ = 0.6 / 2;
}

std::vector<Eigen::Vector2d> ICP::setScan(std::vector<float> &ranges, Pose2D &pos) {
    auto start = std::chrono::system_clock::now();
    int i, ranges_len, clip_min, clip_max, points_len;
    double range_theta;
    uint32_t index;
    std::vector<Eigen::Vector2d> local_points;

    ranges_len = ranges.size();

    if (ranges_len == 0)
        return local_points;

    // 事前にcosSinの計算を済ませておく
    if ((int)cosSin.size() < ranges_len) {
        cosSin.resize(ranges_len);
        double angle_increment = 2 * M_PI / (double)ranges_len;
        std::cout << "icp: init sinCos " << angle_increment << std::endl;
        std::cout << "icp: ranges_len " << ranges_len << std::endl;
        for (i = 0; i < ranges_len; ++i) {
            range_theta = angle_increment * (double)i + lidar_pos.yaw;
            cosSin[i][0] = cos(range_theta);
            cosSin[i][1] = sin(range_theta);
        }
    }

    // 時系列座標変換
    static double prev_ts = get_time_sec();
    double pos_ekf_now_ts = get_time_sec();
    double pos_ekf_dt = (pos_ekf_now_ts - prev_ts) / (double)ranges_len;
    // 過去の時系列順に調べる
    std::vector<int> pos_ekf_id;
    for (int i = pos_ekf_i - 1; 0 <= i; --i) {
        if (pos_ekf[i].time_stamp >= prev_ts) {
            pos_ekf_id.emplace_back(i);
        }
    }
    for (int i = (EKF_BUFF-1); pos_ekf_i<=i; --i) {
        if (pos_ekf[i].time_stamp >= prev_ts) {
            pos_ekf_id.emplace_back(i);
        }
    }
    prev_ts = pos_ekf_now_ts;
    int len_pos_ekf = pos_ekf_id.size();
    int i_pos_ekf = 0;
    if (len_pos_ekf == 0) {
        std::cout << "icp: not found odom" << std::endl;
        return local_points;
    }else{
        // for (int i = 0; i < len_pos_ekf;++i){
        //     std::cout << pos_ekf[pos_ekf_id[i]].time_stamp - pos_ekf[pos_ekf_id[0]].time_stamp << std::endl;
        // }
        // std::cout << std::endl;
    }

    // 距離からグローバル２次元座標へ変換
    clip_min = MIN(angle_min, ranges_len);
    clip_max = MIN(angle_max, ranges_len);
    std::vector<Eigen::Vector2d> points;
    Eigen::Vector2d p;
    Eigen::Rotation2Dd R(pos.yaw);
    for (i = (ranges_len-1); 0 <= i; --i) {
        pos_ekf_now_ts -= pos_ekf_dt;
        if (i_pos_ekf < (len_pos_ekf - 1)) {
            if (pos_ekf[pos_ekf_id[i_pos_ekf]].time_stamp > pos_ekf_now_ts) {
                i_pos_ekf++;
                // std::cout << pos_ekf_id[i_pos_ekf] << ", " << i << std::endl;
            }
        }
        pos_ekf[pos_ekf_id[i_pos_ekf]].xy -= pos_ekf[pos_ekf_id[i_pos_ekf]].v * pos_ekf_dt;
        pos_ekf[pos_ekf_id[i_pos_ekf]].yaw -= pos_ekf[pos_ekf_id[i_pos_ekf]].vyaw * pos_ekf_dt;
        // 範囲外
        if(i < clip_min)
            break;
        if (clip_max < i)
            continue;
        // 数値でない
        if (!std::isfinite(ranges[i]))
            continue;
        p = ranges[i] * cosSin[i] + lidar_pos.xy;
        // ロボット付近の店群を削除
        if(abs(p[0]) < robot_edge_2_ || abs(p[1]) < robot_edge_2_)
            continue;
#ifdef YAW_INTERPOL
        Eigen::Rotation2Dd R(pos_ekf[pos_ekf_id[i_pos_ekf]].yaw);
        p = R * p + pos_ekf[pos_ekf_id[i_pos_ekf]].xy;
#else
        p = pos_ekf[pos_ekf_id[i_pos_ekf]].R * p + pos_ekf[pos_ekf_id[i_pos_ekf]].xy;
#endif
        points.push_back(p);
    }

    // ボクセルグリッド
    std::map<uint32_t, Voxel> voxel_grid;  // 重複なし配列
    for (auto &point : points) {
        index = GRID_GET_INDEX(point[0], point[1]);  // 配列の場所
        if (GRID_OUT(index))
            continue;
        auto itr = voxel_grid.find(index);  // 以前に登録してないか探す
        if (itr != voxel_grid.end()) {      // 見つかった
            itr->second.p += point;         // 重心に加算
            itr->second.len++;              // 加算した回数を保存
        } else {                            // 見つからなかったのでボクセルを登録
            Voxel voxel;
            voxel.p = point;
            voxel.len = 1;
            voxel_grid.insert(std::pair<uint32_t, Voxel>(index, voxel));
        }
        // 代入した場所を記憶しておく
    }
    std::vector<int> wall_ids;
    points.clear();
    for (auto &voxel : voxel_grid) {
        voxel.second.p /= (double)voxel.second.len;  // ボクセルの重心
        if (wall_grid[voxel.first] != 0xff) {
            points.push_back(voxel.second.p);
            wall_ids.push_back(wall_grid[voxel.first]);
        }
    }
    voxel_grid.clear();
    points_len = points.size();
    if (points_len <= 10) {
        std::cout << "icp: wall is not found" << std::endl;
        return points;
    }

    // ローカル座標へ変換
    local_points.reserve(points_len);
    Eigen::Rotation2Dd R_inv = R.inverse();
    for (auto &point : points) {
        local_points.push_back(R_inv * (point - pos.xy));
    }

    optimizePose(pos, local_points, wall_ids);

    // double sum = 0;
    // double err;
    // for (auto &voxel_index : voxel_indexs) {
    //     auto &v = voxel_grid[voxel_index];
    //     if (v.wall_id != -1) {
    //         auto &wall = walls[v.wall_id];
    //         err = wall.n.dot(v.p - wall.p);
    //         sum += err * err;
    //     }
    // }

    // 壁の確認
#ifdef VIEW_MAP
    points.clear();
    double x, y;
    for (x = -GRID_RANGE; x < GRID_RANGE; x += GRID_UNIT) {
        for (y = -GRID_RANGE; y < GRID_RANGE; y += GRID_UNIT) {
            index = GRID_GET_INDEX(x, y);
            if (!GRID_OUT(index) && wall_grid[index] != 0xff) {
                points.push_back(Eigen::Vector2d(x, y));
            }
        }
    }
#endif
    auto end = std::chrono::system_clock::now();
    int elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    

    // std::cout  <<elapsed << std::endl;

    //
    // std::cout << clip_min << ", " << clip_max << ", " << ranges_len << std::endl;
    // std::cout << sum << std::endl;
    return points;
}

void ICP::setEKF(Point2D &pos){
    pos_ekf[pos_ekf_i].time_stamp = get_time_sec();
    pos_ekf[pos_ekf_i].xy(0) = pos.x;
    pos_ekf[pos_ekf_i].xy(1) = pos.y;
    pos_ekf[pos_ekf_i].v(0) = pos.vx;
    pos_ekf[pos_ekf_i].v(1) = pos.vy;
    pos_ekf[pos_ekf_i].yaw = pos.yaw;
    pos_ekf[pos_ekf_i].vyaw = pos.vyaw;
#ifndef YAW_INTERPOL
    pos_ekf[pos_ekf_i].R = Eigen::Rotation2Dd(pos.yaw);
#endif
    if (++pos_ekf_i == EKF_BUFF){
        pos_ekf_i = 0;
    }
}

void ICP::makeMap(bool red){
    std::cout << "icp: makeMap: " << red << std::endl;
    // 壁のリセット
    wall_grid.fill(0xff);
    flip_ = red ? 1 : -1;
    //  横壁の登録
    addWall(0.038, 3.462, 0.0, 0.0, 0.4); // 端
    addWall(0.038, 3.462, 6.924, 6.924, 0.4); // 端
    addWall(1.526, 2.526, 1.481, 1.481, 0.4); // 中心
    addWall(0.626, 1.626, 2.481, 2.481, 0.4); // 中心
    addWall(0.038, 0.588, 2.981, 2.981, 0.4); // 中心
    addWall(1.526, 2.526, 3.481, 3.481, 0.4); // 中心
    addWall(0.626, 1.626, 4.481, 4.481, 0.4); // 中心
    addWall(1.526, 2.526, 5.481, 5.481, 0.4); // 中心
    //  縦壁の登録
    addWall(0.038, 0.038, -0.038, 6.962, 0.2); // 端
    addWall(3.462, 3.462, -0.038, 6.962, 0.2); // 端
    addWall(0.607, 0.607, 1.462, 5.5, 0.4); // 中心
    addWall(2.545, 2.545, 1.462, 5.5, 0.4); // 中心

    // blackエリアの追加
    // ToDo 3箇所のオブジェクトがあるエリアを指定
    eraseMap(0.0, 0.738, 0.0, 0.7);
    eraseMap(0.468, 0.738, 6.224, 6.674);
    eraseMap(2.762, 3.102, 6.654, 6.924);
}

bool ICP::addWall(double x1, double x2, double y1, double y2, double width) {
    int i, j, x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    uint32_t index;
    uint8_t wall_id = 0xff;
    int index_width = width / GRID_UNIT / 2.f;
    Wall wall;
    bool add_wall = false;
    // 逆コート反転
    x1 *= flip_;
    x2 *= flip_;

    if ((x1 == x2 && y1 == y2) || width == 0 || walls.size() == 0xff) {
        return false;
    } else if (x1 == x2) {  // 縦壁
        index = GRID_GET_N(x1);
        x_min = index - index_width;
        x_max = index + index_width;
        y_min = GRID_GET_N(MIN(y1, y2));
        y_max = GRID_GET_N(MAX(y1, y2));
        wall.n = Eigen::Vector2d(1, 0);  // 法線ベクトル
    } else if (y1 == y2) {               // 横壁
        index = GRID_GET_N(y1);
        x_min = GRID_GET_N(MIN(x1, x2));
        x_max = GRID_GET_N(MAX(x1, x2));
        y_min = index - index_width;
        y_max = index + index_width;
        wall.n = Eigen::Vector2d(0, 1);  // 法線ベクトル
    }
    // 壁の登録
    wall_id = walls.size();
    wall.p = Eigen::Vector2d(x1, y1);  // 壁上の点
    wall.W = wall.n * wall.n.transpose();
    // 四角形を壁のIDを入れる
    // ToDo: 最短の壁を設定するようにする
    for (j = y_min; j < y_max; ++j) {
        for (i = x_min; i < x_max; ++i) {
            index = i + j * GRID_SIZE;
            if (!GRID_OUT(index)) {
                auto &id = wall_grid[index];
                // すでに壁が登録されてる
                if(id != 0xff){
                    auto xy = Eigen::Vector2d(GRID_GET_X(i), GRID_GET_X(j));
                    double prev_d = abs(walls[id].n.dot(walls[id].p - xy));
                    double now_d = abs(wall.n.dot(wall.p - xy));
                    // 前に登録した壁の方が近い
                    if(prev_d < now_d)
                        continue;
                }
                id = wall_id;
                add_wall = true;
            }
        }
    }
    if (add_wall) {                        // 壁を追加できた
        walls.push_back(wall);
    }
    return add_wall;
}

bool ICP::eraseMap(double x1, double x2, double y1, double y2){
    int x_min, x_max, y_min, y_max, i, j;
    uint32_t index;
    bool clear = false;
    x1 *= flip_;
    x2 *= flip_;
    x_min = GRID_GET_N(MIN(x1, x2));
    x_max = GRID_GET_N(MAX(x1, x2));
    y_min = GRID_GET_N(MIN(y1, y2));
    y_max = GRID_GET_N(MAX(y1, y2));
    for (j = y_min; j < y_max; ++j) {
        for (i = x_min; i < x_max; ++i) {
            index = i + j * GRID_SIZE;
            if (!GRID_OUT(index)) {
                wall_grid[index] = 0xff;
                clear = true;
            }
        }
    }
    return clear;
}

// ガウス-ニュートン法による非線形最適化
double ICP::optimizePose(Pose2D &robot_pos, const std::vector<Eigen::Vector2d> &points, const std::vector<int> &wall_ids) {
    const static int MAX_STEPS = 50;
    const static double evthre = 0.000001;
    double prevErr = 1000000;
    Pose2D pose;
    pose.xy(0) = robot_pos.xy(0);
    pose.xy(1) = robot_pos.xy(1);
    pose.yaw = robot_pos.yaw;
    for (int i = 0; i < MAX_STEPS; ++i) {
        Pose2D npose;
        double curErr = calGaussNewton(pose, points, wall_ids, npose);
        if (std::isnan(pose.xy(0)) || std::isnan(pose.xy(1)) || std::isnan(pose.yaw))
            break;

        if (abs(prevErr - curErr) <= evthre) {  // 収束
            if (curErr < prevErr) {
                pose = npose;
                prevErr = curErr;
            }
            break;
        }
        if (curErr < prevErr) {
            pose = npose;
            prevErr = curErr;
        } else
            break;
    }
    if (!std::isnan(pose.xy(0)) && !std::isnan(pose.xy(1)) && !std::isnan(pose.yaw)){
        robot_pos.xy(0) = pose.xy(0);
        robot_pos.xy(1) = pose.xy(1);
        robot_pos.yaw = pose.yaw;
        // initPos(pose);
    }else{
        std::cout << "icp: failed optimizePose" << std::endl;
    }

    return (prevErr);
}

// 推定位置pose、現在スキャン点群curLps、参照スキャン点群refLps
double ICP::calGaussNewton(const Pose2D &pose, const std::vector<Eigen::Vector2d> &points, const std::vector<int> &wall_ids, Pose2D &newPose) {
    // 回転行列
    Eigen::Rotation2Dd R(pose.yaw);

    Eigen::Matrix3d JWJ = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Vector3d JWe = Eigen::Vector3d::Zero(3);
    double totalErr = 0;
    int points_len = points.size();
    for (int i = 0; i < points_len; ++i) {
        auto &point = points[i];
        auto &wall = walls[wall_ids[i]];

        // if (rlp->type != LINE)  // 法線のない点は使わない
        //     continue;

        // if (hasOutliers)
        // addNoise(i, cx, cy);  // 外れ値テスト
        // 回転
        Eigen::Vector2d r = R * point;
        // 対応点との距離
        Eigen::Vector2d e = (r + pose.xy) - wall.p;
        // 壁との垂直距離
        double err = e.transpose() * wall.W * e;

        double drho = 1;
#ifdef ROBUST
        drho = robustWeightHuber(err);
//      drho = robustWeightTukey(err);
#endif

        // ヤコビアン
        Eigen::Matrix<double, 2, 3> J;
        J(0, 0) = 1;
        J(0, 1) = 0;
        J(0, 2) = -r(1);
        J(1, 0) = 0;
        J(1, 1) = 1;
        J(1, 2) = r(0);

        Eigen::Matrix<double, 3, 2> A = drho * J.transpose() * wall.W;

        JWJ += A * J;
        JWe += A * e;
        totalErr += drho * err;
    }

    Eigen::Vector3d d = -JWJ.inverse() * JWe;
    newPose.xy(0) = pose.xy(0) + d(0);
    newPose.xy(1) = pose.xy(1) + d(1);
    newPose.yaw = normalizeAngle(pose.yaw + d(2));

    //  printf("newPose=(%g %g %g), totalErr=%g\n", newPose.tx, newPose.ty, newPose.th, totalErr);

    return (totalErr);
}

// Huber robust function
double ICP::robustWeightHuber(double e) {
    double drho;
    if (e < evlimit*evlimit)
        drho = 1;
    else 
        drho = evlimit/sqrt(e);

    return(drho);
}

// Tukey robust function
double ICP::robustWeightTukey(double e) {
    double drho;
    if (e < evlimit*evlimit) {
        double r = e/(evlimit*evlimit);
        drho = (1-r)*(1-r);
    }
    else 
        drho = 0;

    return(drho);
}