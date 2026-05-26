#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "haru25_msgs/msg/can_array.hpp"
#include "haru25_msgs/msg/can.hpp"
#include "haru25_msgs/msg/pos.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

#define ENCODER_PPR (2048)//defineは値に()をつける　分解能2048
#define ENCODER_RADIUS (-0.024)
#define METER_2_COUT (ENCODER_PPR * 4.0 / (2.0 * M_PI * ENCODER_RADIUS))//1m進むときのエンコーダーのカウント数

double normalizeAngle(double theta) {
    return theta - 2 * M_PI * floor((theta + M_PI) / (2 * M_PI));
}

class TempNode : public rclcpp::Node {
    // 上り　gazevo→上位レイヤー
    rclcpp::Publisher<haru25_msgs::msg::CANArray>::SharedPtr can_pub_;//センサー
    rclcpp::Publisher<haru25_msgs::msg::Pos>::SharedPtr pos_pub_;//センサー

    // 下り 上位レイヤー→gazebo
    rclcpp::Subscription<haru25_msgs::msg::CANArray>::SharedPtr can_sub_;//モーター
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;  // モーター

    haru25_msgs::msg::CANArray can_array_;

   public:
    TempNode()
        : Node("haru25_gazebo") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&TempNode ::odom_cb, this, _1)); // グローバル座標をgazeboから受け取る
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&TempNode ::imu_cb, this, _1));

        can_pub_= this->create_publisher<haru25_msgs::msg::CANArray>("can/rx", 10);
        pos_pub_ = this->create_publisher<haru25_msgs::msg::Pos>("true", 10);

        can_sub_= this->create_subscription<haru25_msgs::msg::CANArray>(//モーターの速度をguiから受け取る
            "can/tx", 10, std::bind(&TempNode ::can_cb, this, _1));//txが上位→下位
        wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 10);

        haru25_msgs::msg::CAN can;
        can.id = 0xff;
        can.data.resize(9, 0);
        can_array_.array.push_back(can);
    }

   private:
   //gazevo→上位レイヤー　受け取ったグローバル座標をローカル座標に変換してcanにいれる.それをcan_arrayにいれてcan_pub_でpublishする(odom_sub_とcan_pub_がセット)
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {//受信したときに呼ばれる関数
        double x = msg->pose.pose.orientation.x;//x=(*msg).pose.pose.orientation.x
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;

        double sqw = w * w;
        double sqx = x * x;
        double sqy = y * y;
        double sqz = z * z;

        double yaw = atan2(2.0 * (x * y + z * w), (sqx - sqy - sqz + sqw));
        static double prev_yaw=yaw;
        
        static auto pose_x = msg->pose.pose.position.x;
        static auto pose_y = msg->pose.pose.position.y;

        double dx = msg->pose.pose.position.x - pose_x;
        double dy = msg->pose.pose.position.y - pose_y;
        pose_x = msg->pose.pose.position.x;
        pose_y = msg->pose.pose.position.y;

        double c = cos(-yaw);
        double s = sin(-yaw);

        double local_dx = dx*c - dy*s;
        double local_dy = dx*s + dy*c;

        can_array_.array[0].data[0] = -local_dx * METER_2_COUT;  // 位置の差分をいれる
        can_array_.array[0].data[1] = local_dy * METER_2_COUT;

        double dyaw = normalizeAngle(yaw-prev_yaw);
        prev_yaw = yaw;

        can_array_.array[0].data[2] = dyaw/2;

        can_pub_->publish(can_array_);

        // ロボット自己位置を出力
        haru25_msgs::msg::Pos pos;
        pos.x = msg->pose.pose.position.x;
        pos.y = msg->pose.pose.position.y;
        pos.yaw = yaw;
        pos.vx = msg->twist.twist.linear.x;
        pos.vy = msg->twist.twist.linear.y;
        pos.vyaw = msg->twist.twist.angular.z;
        pos_pub_->publish(pos);

    }
    //上位レイヤー→gazebo　guiから受け取ったモーターの速度をそのままwheel_pub_でpublishする（can_sub_とwheel_pub_がセット）
    void can_cb(const haru25_msgs::msg::CANArray::SharedPtr msg) {//受信したときに呼ばれる関数

        auto flt_msg = std_msgs::msg::Float64MultiArray();
        flt_msg.data.resize(4);//4つの要素を持つ配列を作成

        for(auto &can:msg->array){//受信したデータを1つずつ処理 for can in arrayと同じ
            if(can.id < 4 && can.data.size() == 2){//idが4未満のときとdataのサイズが2のとき(dataのサイズで位置・速度・電流のどれかを判断)
                flt_msg.data[can.id] = can.data[1];//モーターの速度をいれる
                if(can.id == 3){
                    wheel_pub_->publish(flt_msg);
                }
            }
        }
    }
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg){
        can_array_.array[0].data[3] = msg->linear_acceleration.x;
        can_array_.array[0].data[4] = msg->linear_acceleration.y;
        can_array_.array[0].data[5] = msg->linear_acceleration.z;
        can_array_.array[0].data[6] = msg->angular_velocity.x;
        can_array_.array[0].data[7] = msg->angular_velocity.y;
        can_array_.array[0].data[8] = msg->angular_velocity.z;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TempNode>());
    rclcpp::shutdown();
    return 0;
}