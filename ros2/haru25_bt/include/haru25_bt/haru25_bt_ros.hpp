#pragma once

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "haru25_msgs/msg/plan.hpp"
#include "haru25_msgs/msg/can_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "haru25_msgs/msg/pos.hpp"
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp/bt_factory.h"
// #include "behaviortree_cpp/loggers/groot2_publisher.h"

//buttons
#define A 0
#define B 1
#define X 2
#define Y 3
#define LB 4
#define RB 5
#define SCREEN 6
#define MENU 7
#define XBOX 8
#define L3 9
#define R3 10
#define UPROAD 11
//axes
#define LX 0
#define LY 1
#define LT 2
#define RX 3
#define RY 4
#define RT 5
#define LEFT_RIGHT 6
#define UP_DWUN 7

#define VY_MAX 1
#define VX_MAX 1
#define OMEGAMAX (6.28 * 0.25)
#define DT 0.01

//Air id haru25_bt_ros.cpp haru25_bt.cpp
// LEDの順番
#define BALLCATCH_RELEASE 1  
#define ERASER 3
#define SHOT 2
#define BALLUP_DOWN 0

//Air haru25_bt.cpp true or false
#define BALL_CATCH false
#define BALL_RELEASE true  
#define ERASER_CATCH true
#define ERASER_RELEASE false 
#define ERASER_SHOT true
#define BALL_UP false
#define BALL_DOWN true


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

//tape_led
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace BT;

// 状態推移
typedef enum {
    WAIT,          // ボタンが押されるのを待っている
    AUTO,          // 受け渡し場所で待つ
    MANUAL,  // 受け渡し場所へ行く途中
} State;

class HaruBt : public rclcpp::Node {
    rclcpp::Publisher<haru25_msgs::msg::Plan>::SharedPtr plan_pub_;//下り　上位→下位
    rclcpp::Publisher<haru25_msgs::msg::CANArray>::SharedPtr can_pub_;//下り　上位→下位
    rclcpp::Publisher<haru25_msgs::msg::Pos>::SharedPtr init_pub_;

    rclcpp::Subscription<haru25_msgs::msg::Plan>::SharedPtr plan_sub_;//下り　上位→下位
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<haru25_msgs::msg::Pos>::SharedPtr pos_sub_;

    sensor_msgs::msg::Joy::SharedPtr joy_;

    rclcpp::TimerBase::SharedPtr timer_;

    haru25_msgs::msg::CANArray can_array_;

    // 自動運転
    bool plan_isReached_ = false;

    //フィールド切り替え
    bool red_ = true;

    // 状態
    State state_ = WAIT;

    // 手動操作時の目標角度
    float manual_yaw_ = 0;

    // 初期位置
    haru25_msgs::msg::Pos init_pos_;

    // BehaviorTree
    std::unique_ptr<Tree> bt_tree_;
    BehaviorTreeFactory *bt_factory_;
    std::string bt_path_;
    // std::unique_ptr<Groot2Publisher> bt_groot_pub_;

    float max_a_;
    bool air_a_ = false;
    bool air_b_ = false;
    bool air_x_ = false;
    bool air_y_ = false;
    bool led_ = false;

   public:
    // 自己位置
    haru25_msgs::msg::Pos pos;
    HaruBt(BehaviorTreeFactory *factory);
    void air_write(uint8_t id, bool on);
    void log(const char *str, ...);
    void plan_set(int path_i);
    bool plan_isReached();
    void init_pos();
    void led_write(int color);
   private:
    void make_tree(std::string tree_name);
    void timer_cb();
    void manual();
    void plan_cb(const haru25_msgs::msg::Plan::SharedPtr msg);
    void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg);
    void pos_cb(const haru25_msgs::msg::Pos::SharedPtr msg);
};
