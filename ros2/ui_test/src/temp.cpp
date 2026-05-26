#include <functional>
#include <memory>
#include <string>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ui_test/Common.h"
#include "ui_test/pid.h"

#define SQRT_1_2 (1.f / M_SQRT2)
#define VELOCITY_LIMIT_ANGLAR (6.28 * 2.0) //pidの最大出力 90度回転（L2,R2）のとき　あんまり変更しない　ここを変更するとOMEGAMAXやゲインにも影響が出る

PID pid_yaw(10.0, 0, 0.5, VELOCITY_LIMIT_ANGLAR);

using std::placeholders::_1;
using namespace std::chrono_literals;

class TempNode : public rclcpp::Node {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub_;
    // ホイールの速度をPublish
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr str_sub_;

    // コントローラーをSubscribe
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_;

    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy::SharedPtr joy_msg_;

    float yaw_ = 0;

   public:
    TempNode()
        : Node("temp") {
        str_pub_ = this->create_publisher<std_msgs::msg::String>("pubpub", 10);
        // ホイールの速度をPublish
        wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 10);


        str_sub_ = this->create_subscription<std_msgs::msg::String>(
            "sub", 10, std::bind(&TempNode ::str_cb, this, _1));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TempNode::joy_cb, this, _1));

        imu_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TempNode::imu, this, _1));
    
        timer_ = this->create_wall_timer(10ms, std::bind(&TempNode::timer_cb, this));
    }

   private:
    void timer_cb() {
        if(joy_msg_ == nullptr)
            return;
        static auto m_start = std::chrono::system_clock::now();
        auto m_now = std::chrono::system_clock::now();
        int nowtimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(m_now - m_start).count();
        // std::cout << now << std::endl;


        float joy_x = joy_msg_->axes[0];
        float joy_y = joy_msg_->axes[1];
        bool btn = joy_msg_->buttons[4];
        bool btn2 = joy_msg_->buttons[5];
        bool btn3 = joy_msg_->buttons[1];
        bool btn4 = joy_msg_->buttons[2];
        

        // float vx_local = -joy_x;
        // float vy_local = joy_y;
        float vw = 0;
        if(btn==1){
            vw = btn;
        }
        if(btn2==1){
            vw = -btn2;
        }
        // float vw = btn;

        static float aim_yaw = 0;
        aim_yaw += vw * 0.01;
        static unsigned long push_90_time_stamp = 0;
        // unsigned long nowtimestamp = 0.01;//現在時刻をミリ秒で取得
        if( (nowtimestamp-push_90_time_stamp) > 1000 ){//ボタンをおした瞬間だけみる
        RCLCPP_INFO(this->get_logger(), "%d %d",btn3,btn4);

        if(btn3){
          aim_yaw += M_PI/2;
          push_90_time_stamp = nowtimestamp;
        }else if(btn4){
          aim_yaw += -M_PI/2;
          push_90_time_stamp = nowtimestamp;
        }
      }

        vw = pid_yaw.update(normalizeAngle(aim_yaw-yaw_));//コメントアウトした②


        float wheel_1_radius_ = 20;

        float c = cosf(-yaw_);
        float s = sinf(-yaw_);
        float vx_local = -joy_x * c - joy_y * s;
        float vy_local = -joy_x * s + joy_y * c;

        float v1 = vx_local * SQRT_1_2 - vy_local * SQRT_1_2 - vw;
        float v2 = vx_local * SQRT_1_2 + vy_local * SQRT_1_2 - vw;
        float v3 = -vx_local * SQRT_1_2 + vy_local * SQRT_1_2 - vw;
        float v4 = -vx_local * SQRT_1_2 - vy_local * SQRT_1_2 - vw;

        auto flt_msg = std_msgs::msg::Float64MultiArray();
        flt_msg.data.push_back(v1*wheel_1_radius_);//Pythonのappend
        flt_msg.data.push_back(v2*wheel_1_radius_);
        flt_msg.data.push_back(v3*wheel_1_radius_);
        flt_msg.data.push_back(v4*wheel_1_radius_);
        wheel_pub_->publish(flt_msg);

        // RCLCPP_INFO(this->get_logger(), "つかれた");
        // auto msg = std_msgs::msg::Float64MultiArray();
        // msg.data.push_back(-10);//Pythonのppend
        // msg.data.push_back(10);
        // msg.data.push_back(-10);
        // msg.data.push_back(10);
        // wheel_pub_->publish(msg);
        // auto message = std_msgs::msg::String();
        // message.data = "つかれた2 ";
        // str_pub_->publish(message);
    }
    void str_cb(const std_msgs::msg::String::SharedPtr msg) {
        auto message = std_msgs::msg::String();
        message.data = "I heard " + msg->data;
        str_pub_->publish(message);
    }
    void imu(const nav_msgs::msg::Odometry::SharedPtr msg){
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;

        double sqw = w * w;
        double sqx = x * x;
        double sqy = y * y;
        double sqz = z * z;

        yaw_ =  atan2(2.0 * (x * y + z * w), (sqx - sqy - sqz + sqw));
    }
    void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg){
        joy_msg_ = msg;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);// rosの初期化
    std::cout << "init" << std::endl;
    rclcpp::spin(std::make_shared<TempNode>());// rosを実行
    std::cout << "end" << std::endl;
    rclcpp::shutdown();// rosをシャットダウン
    return 0;
}