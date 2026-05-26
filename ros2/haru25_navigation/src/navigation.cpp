#include <tf2_ros/transform_broadcaster.h>

#include <functional>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "haru25_navigation/ekf.hpp"
#include "haru25_navigation/icp.hpp"
#include "haru25_navigation/omni_drive.hpp"
#include "haru25_navigation/common.hpp"
#include "haru25_navigation/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "haru25_msgs/msg/can_array.hpp"
#include "haru25_msgs/msg/can.hpp"
#include "haru25_msgs/msg/plan.hpp"
#include "haru25_msgs/msg/pos.hpp"
#include "haru25_navigation/pid.hpp"
#include "haru25_navigation/plan.hpp"
#include "haru25_navigation/purepursuit.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


// 重要パラメーター
#define VELOCITY_LIMIT_LINEAR 0.75//0.75
#define VELOCITY_LIMIT_ANGLAR 3.14//3.14

// #define DEBUG_PURSUIT

using std::placeholders::_1;
using namespace std::chrono_literals;

typedef enum {
    STOP=0,
    MANUAL,
    AUTO,
} Mode;

class NavNode : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    
    rclcpp::Subscription<haru25_msgs::msg::CANArray>::SharedPtr can_sub_;//上り　下位→上位
    rclcpp::Publisher<haru25_msgs::msg::CANArray>::SharedPtr can_pub_;//下り　上位→下位

    rclcpp::Subscription<haru25_msgs::msg::Plan>::SharedPtr plan_sub_;//下り　上位→下位
    rclcpp::Publisher<haru25_msgs::msg::Plan>::SharedPtr plan_pub_;//下り　上位→下位

    rclcpp::Subscription<haru25_msgs::msg::Pos>::SharedPtr pos_sub_;
    rclcpp::Publisher<haru25_msgs::msg::Pos>::SharedPtr pos_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    rclcpp::TimerBase::SharedPtr timer_;

    ICP icp_;//private変数
    EKF ekf_;
    OmniDrive omni_;
    Odometry odom_;
    Pose2D pos_odom_;
    Point2D pos_ekf_;
    Plan plan_;
    PurePursuit pure_pursuit_;
    PID pid_yaw_;  // 角度がフラフラするとき角度のPDゲインを上げる。Pゲインを上げてみてDゲインは発振したら上げる。
    PID pid_foot_;  // 角度PIDが目標角度までいかないとき、PとIゲインを上げる
    PID pid_path_;  // 

    Mode mode_;
    Pose2D ref_manual_;//手動操作のときの目標速度x,y,yaw

    haru25_msgs::msg::CANArray can_array_;

    bool pub_lidar_conv_;
    builtin_interfaces::msg::Time frame_ts_;
    bool sub_lidar_ = false;
    bool sub_odom_ = false;

    public:
    NavNode()//コンストラクタ 一回だけ実行される
        : Node("navigation") , 
          omni_(0.2, 0.05),//gazeboのときは0.05 実機のときは-0.05
          odom_(0.024, -0.024),
          mode_(STOP),
          pid_yaw_(2.0, 0, 0.1, VELOCITY_LIMIT_ANGLAR),//2.0, 0, 0.1, VELOCITY_LIMIT_ANGLAR
          pid_foot_(5.0, 0., 1.0, VELOCITY_LIMIT_LINEAR),//5.0, 0., 1.0, VELOCITY_LIMIT_LINEAR
          pid_path_(0.1, 0.0, 0.01, VELOCITY_LIMIT_LINEAR),
          plan_(),
          pure_pursuit_(&plan_, &pid_yaw_, &pid_foot_, &pid_path_)
          {
        // 経路のデータの読み込み
        std::string plan;
        this->declare_parameter("plan_path", "/home/yui/ros2_ws/src/haru25/ros2/haru25/plan/csv/plan.csv");
        this->get_parameter("plan_path", plan);

        // 最大速度
        double max_vel_linear, max_vel_angular;
        this->declare_parameter("max_vel_angular", VELOCITY_LIMIT_ANGLAR);
        this->get_parameter("max_vel_angular", max_vel_angular);
        this->declare_parameter("max_vel_linear", VELOCITY_LIMIT_LINEAR);
        this->get_parameter("max_vel_linear", max_vel_linear);

        // 最大速度
        double min_vel_linear, min_vel_angular;
        this->declare_parameter("min_vel_angular", 0.3);
        this->get_parameter("min_vel_angular", min_vel_angular);
        this->declare_parameter("min_vel_linear", 0.1);
        this->get_parameter("min_vel_linear", min_vel_linear);

        // 角度PID制御のゲイン 配列のため、vectorで取得
        
        std::vector<double> pid_yaw_gain;
        this->declare_parameter("pid_yaw", pid_yaw_gain);
        this->get_parameter("pid_yaw", pid_yaw_gain);
        if(pid_yaw_gain.size() == 3){
            pid_yaw_.setGain(pid_yaw_gain[0], pid_yaw_gain[1], pid_yaw_gain[2], max_vel_angular);
        }else{
            RCLCPP_ERROR(this->get_logger(), "pid_yaw size is not 3");
        }

        // 並進PID制御のゲイン

        std::vector<double> pid_foot_gain;
        double max_vel_linear_pid;
        this->declare_parameter("pid_foot", pid_foot_gain);
        this->get_parameter("pid_foot", pid_foot_gain);
        this->declare_parameter("max_vel_linear_pid", max_vel_linear_pid);
        this->get_parameter("max_vel_linear_pid", max_vel_linear_pid);
        if(pid_foot_gain.size() == 3){
            pid_foot_.setGain(pid_foot_gain[0], pid_foot_gain[1], pid_foot_gain[2], max_vel_linear_pid);
        }else{
            RCLCPP_ERROR(this->get_logger(), "pid_foot size is not 3");
        }

        // 経路追従PID制御のゲイン
        std::vector<double> pid_path_gain;
        this->declare_parameter("pid_path", pid_path_gain);
        this->get_parameter("pid_path", pid_path_gain);
        if(pid_path_gain.size() == 3){
            pid_path_.setGain(pid_path_gain[0], pid_path_gain[1], pid_path_gain[2], max_vel_linear);
        }else{
            RCLCPP_ERROR(this->get_logger(), "pid_path size is not 3");
        }

        // 自動運転のパラメーター
        double path_goal_distance, path_goal_angle, path_pid_distance;
        this->declare_parameter("path_goal_distance", 0.05);
        this->declare_parameter("path_goal_angle", DEG2RAD(10.));
        this->declare_parameter("path_pid_distance", 0.1);
        this->get_parameter("path_goal_distance", path_goal_distance);
        this->get_parameter("path_goal_angle", path_goal_angle);
        this->get_parameter("path_pid_distance", path_pid_distance);
        pure_pursuit_.setParam(path_goal_distance,
                               path_goal_angle,
                               path_pid_distance,
                               max_vel_linear,
                               max_vel_angular,
                               min_vel_linear,
                               min_vel_angular);

        //タイヤの半径のパラメーター
        double wheel_radius;
        this->declare_parameter("wheel_radius", 0.05);//初期値
        this->get_parameter("wheel_radius", wheel_radius);//値を取得
        omni_.set_wheel_1_radius_(wheel_radius);//値をセット

        // robomasのid
        int id1, id2, id3, id4;
        this->declare_parameter("id1", 0);
        this->declare_parameter("id2", 1);
        this->declare_parameter("id3", 2);
        this->declare_parameter("id4", 3);
        this->get_parameter("id1", id1);
        this->get_parameter("id2", id2);
        this->get_parameter("id3", id3);
        this->get_parameter("id4", id4);

        //0
        haru25_msgs::msg::CAN c1, c2, c3, c4;//idとdataの配列
        c1.id = id1;//このidはrobomasのidと一つずれてる　//1
        c1.data.push_back(20);//モーターの最大電流 dataは位置なら3つ、速度なら2つ、電流なら1つの配列
        c1.data.push_back(0);//モーターの目標速度 タイヤ右上　第一象限
        can_array_.array.push_back(c1);
        //1
        c2.id = id2;//3
        c2.data.push_back(20);//モーターの最大電流 dataは位置なら3つ、速度なら2つ、電流なら1つの配列
        c2.data.push_back(0);//モーターの目標速度　タイヤ左上　第二象限
        can_array_.array.push_back(c2);
        //2
        c3.id = id3;//2
        c3.data.push_back(20);//モーターの最大電流 dataは位置なら3つ、速度なら2つ、電流なら1つの配列
        c3.data.push_back(0);//モーターの目標速度　タイヤ左下　第三象限
        can_array_.array.push_back(c3);
        //3
        c4.id = id4;//0
        c4.data.push_back(20);//モーターの最大電流 dataは位置なら3つ、速度なら2つ、電流なら1つの配列
        c4.data.push_back(0);//モーターの目標速度　タイヤ右下　第四象限
        can_array_.array.push_back(c4);

        this->declare_parameter("pub_lidar_conv", false);//初期値
        this->get_parameter("pub_lidar_conv", pub_lidar_conv_);//値を取得

        // .csvファイルの読み込み
        int plan_size = plan_.readPlan(plan);
        if(plan_size == 0){
            RCLCPP_ERROR(this->get_logger(), "Plan read error");
        }else{
            RCLCPP_INFO(this->get_logger(), "Plan read success: %d", plan_size);
        }
        // 自己位置を設定
        odom_.init(3.112, 0.29, 0);
        ekf_.initPos(3.112, 0.29, 0);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&NavNode ::lidar_cb, this, _1));
        
        can_sub_ = this->create_subscription<haru25_msgs::msg::CANArray>("can/rx", 10, std::bind(&NavNode ::can_cb, this, _1));
        can_pub_ = this->create_publisher<haru25_msgs::msg::CANArray>("can/tx", 10);

        plan_sub_ = this->create_subscription<haru25_msgs::msg::Plan>("plan", 10, std::bind(&NavNode ::plan_cb, this, _1));
        plan_pub_ = this->create_publisher<haru25_msgs::msg::Plan>("reach", 10);

        pos_sub_ = this->create_subscription<haru25_msgs::msg::Pos>("init_pos", 10, std::bind(&NavNode ::pos_cb, this, _1));
        pos_pub_ = this->create_publisher<haru25_msgs::msg::Pos>("pos", 10);

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

        if(pub_lidar_conv_)
            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("points", 10);

        
        tfb_.reset();
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 200Hz
        timer_ = this->create_wall_timer(5ms, std::bind(&NavNode::timer_cb, this));//5msごとにtimer_cbを呼び出す
    }

   private:
    void timer_cb() {
        if(!sub_lidar_)
            return;
        float vx = 0, vy = 0, vt = 0;
        float motor_vel[4];
        Pose2D robot_pos;

        if(mode_ == MANUAL){
            robot_pos = pos_odom_;
        }else{
            robot_pos.xy(0) = pos_ekf_.x;
            robot_pos.xy(1) = pos_ekf_.y;
            robot_pos.yaw = pos_ekf_.yaw;
        }
    
        switch (mode_) {
            case AUTO://自動運転のとき
                if(pure_pursuit_.pursuit(robot_pos, vx, vy, vt)){
                    // 停止モードに移行
                    mode_ = STOP;
                    haru25_msgs::msg::Plan plan;
                    plan.mode = haru25_msgs::msg::Plan::AUTO;
                    plan.pos.x = robot_pos.xy(0);
                    plan.pos.y = robot_pos.xy(1);
                    plan.pos.yaw = robot_pos.yaw;
                    plan_pub_->publish(plan);
                }
                break;
            case MANUAL://手動操作のとき　角度pid制御
#ifdef DEBUG_PURSUIT
                pure_pursuit_.pursuit(robot_pos, vx, vy, vt);
                cout << pure_pursuit_.path_fb_error << ", "<<pure_pursuit_.path_fb_vx << ", " << pure_pursuit_.path_fb_vy << endl;
#endif
                vx = ref_manual_.xy[0];
                vy = ref_manual_.xy[1];
                vt = pid_yaw_.update(normalizeAngle(ref_manual_.yaw - robot_pos.yaw));//目標ー現在の角度
                break;
            case STOP://停止のとき
                break;
        }
        omni_.setRobotSpeed(vx, vy, vt, robot_pos.yaw, motor_vel);//vx,vy,vtをモーターの速度に変換、robot_pos.yawはロボットの角度、motor_velはモーターの速度の配列

        for(int i = 0; i < 4; i++){
            can_array_.array[i].data[1] = motor_vel[i];
        }

        can_pub_->publish(can_array_);
        publishTwistStamped(vx, vy, vt);
    }
    void lidar_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        sub_lidar_ = true;
        Pose2D pos;
        pos.xy(0) = pos_ekf_.x;
        pos.xy(1) = pos_ekf_.y;
        pos.yaw = pos_ekf_.yaw;

        auto p = icp_.setScan(msg->ranges, pos);
        // 手動モードでなければ自己位置を更新
        ekf_.setLidar(pos);
            
        // // publishFrame(pos.xy(0), pos.xy(1), pos.yaw);
        if(pub_lidar_conv_){
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = frame_ts_;
            m.ns = "p";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            m.action = visualization_msgs::msg::Marker::MODIFY;
            m.scale.x = m.scale.y = m.scale.z = 0.01;
            m.color.g = m.color.a = 1;

            int p_len = p.size();
            m.points.resize(p_len);
            for (int i = 0; i < p_len; ++i) {
                m.points[i].x = p[i][0];
                m.points[i].y = p[i][1];
                m.points[i].z = 0.1;
            }
            marker_pub_->publish(m);
        }
    }
    void can_cb(const haru25_msgs::msg::CANArray::SharedPtr msg ){
        static float prev_yaw_ = FLT_MAX;
        // float wheel_v[4] = {0};
        // float dyaw = 0;
        for(auto &can : msg->array){
            // 設置エンコーダー
            // if(can.id < 4)
            //     wheel_v[can.id] = can.data[3];
            if(can.id == 0xff && can.data.size() >= 3){
                // dyaw = can.data[2];
                sub_odom_ = true;
                pos_odom_ = odom_.getPos(can.data[0], can.data[1], can.data[2] / 2.f);
                auto p = odom_.getLocal(can.data[0], can.data[1]);
                ekf_.setOdom(p.xy(0), p.xy(1), can.data[2]);
                // カルマンフィルタ
                pos_ekf_ = ekf_.estimate();
                // ロボット自己位置を出力
                publishFrame(pos_ekf_);
                // ICPに自己位置を登録
                icp_.setEKF(pos_ekf_);
            }
        }
        // auto p = odom_.getLocal2(wheel_v);
        // ekf_.setOdom(p.xy(0), p.xy(1), dyaw);
        // // カルマンフィルタ
        // pos_ekf_ = ekf_.estimate();
        // // ロボット自己位置を出力
        // publishFrame(pos_ekf_);
        // // ICPに自己位置を登録
        // icp_.setEKF(pos_ekf_);
    }
    void plan_cb(const haru25_msgs::msg::Plan::SharedPtr msg ){
        switch (msg->mode){
           case haru25_msgs::msg::Plan::AUTO:
               mode_ = AUTO;
            //    RCLCPP_INFO(this->get_logger(), "Plan set: %d, Reverse: %d", msg->path_i, msg->path_reverse);
                path_pub_->publish(make_path(msg));
               break;
           case haru25_msgs::msg::Plan::MANUAL:
               if(mode_ == AUTO)
                    odom_.init(pos_ekf_.x, pos_ekf_.y, pos_ekf_.yaw);
               mode_ = MANUAL;
               ref_manual_.xy[0] = msg->pos.vx;
               ref_manual_.xy[1] = msg->pos.vy;
               ref_manual_.yaw = msg->pos.yaw;
               break;
           case haru25_msgs::msg::Plan::STOP:
               mode_ = STOP;
               break;
       }
    }
    void pos_cb(const haru25_msgs::msg::Pos::SharedPtr msg) {
        if (!sub_lidar_){
            RCLCPP_ERROR(this->get_logger(), "No Lidar DATA");
            return;
        }
        if (!sub_odom_){
            RCLCPP_ERROR(this->get_logger(), "No Odom DATA");
            return;
        }
        icp_.makeMap(msg->x > 0);
        ekf_.initPos(msg->x, msg->y, msg->yaw);
        odom_.init(msg->x, msg->y, msg->yaw);
    }
    // Rviz2に表示するようにTFを送信
    void publishFrame(Point2D &p) {
        haru25_msgs::msg::Pos pos;
        pos.x = p.x;
        pos.y = p.y;
        pos.yaw = p.yaw;
        pos.vx = p.vx;
        pos.vy = p.vy;
        pos.vyaw = p.vyaw;
        pos_pub_->publish(pos);
        geometry_msgs::msg::TransformStamped tmp_tf_stamped;
        // MAP -> Base
        tf2::Quaternion q;
        q.setRPY(0, 0, p.yaw);
        tf2::Transform tmp_tf(q, tf2::Vector3(p.x, p.y, 0.0));
        tmp_tf_stamped.header.frame_id = "map";
        tmp_tf_stamped.header.stamp = this->get_clock()->now();
        tmp_tf_stamped.child_frame_id = "base_link";
        tf2::convert(tmp_tf, tmp_tf_stamped.transform);
        tfb_->sendTransform(tmp_tf_stamped);
        frame_ts_ = tmp_tf_stamped.header.stamp;
    }
    void publishTwistStamped(float vx, float vy, float vt){
        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = "base_link";
        twist.header.stamp = this->get_clock()->now();
        twist.twist.linear.x = vx;
        twist.twist.linear.y = vy;
        twist.twist.angular.z = vt;    
        twist_pub_->publish(twist);
    }
    nav_msgs::msg::Path make_path(const haru25_msgs::msg::Plan::SharedPtr msg){
        std::vector<Point2D> plan = pure_pursuit_.setPlan(msg->path_i, msg->path_reverse, msg->flip);
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->get_clock()->now();
        path.poses.reserve(plan.size());
        for(auto &p:plan){
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.position.z = 0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p.yaw);
            pose.pose.orientation = tf2::toMsg(q);
            path.poses.emplace_back(pose);
        }
        return path;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavNode>());
    rclcpp::shutdown();
    return 0;
}