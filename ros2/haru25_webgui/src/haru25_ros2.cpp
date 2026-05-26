#include "haru25_webgui/haru25_ros2.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// buttons
constexpr int buttommap[16] = {0, 1, 2, 3, 4, 5, -1, -1, 6, 7, 9, 10, -1, -1, -1, -1};
constexpr int axesmap[4] = {0, 1, 3, 4};
#define A 0
#define B 1
#define X 2
#define Y 3
#define LB 4
#define RB 5
#define LT 6
#define RT 7
#define SCREEN_SHOT 0
#define MENU 1
#define L3 2
#define R3 3
#define UP 4
#define DOWN 5
#define LEFT 6
#define RIGHT 7

#define LX 0
#define LY 1
#define RX 2
#define RY 3

#define ROS2_DATA_LEN 256
#define WEB_DATA_LEN 10
#define ROS2_POS_LEN 4*6
#define ROS2_LOG_LEN (ROS2_DATA_LEN - ROS2_POS_LEN)

std::mutex mtx_; // 排他制御用ミューテックス
// 排他処理（ROS2とWebから同時に書き込むことができないようにする）
char ros2_data_[ROS2_DATA_LEN] = {0};//ROS2から来るデータ
int ros2_data_str_len_ = 0;
uint8_t web_data_[WEB_DATA_LEN] = {0};//Webから来るデータ
bool web_receive_ = false;

Haru25_ros2::Haru25_ros2() : Node("haru25_webgui") {
    declare_parameter("www_path", "/home/yui/ros2_ws/src/haru25/ros2/haru25_webgui/www/");
    this->get_parameter("www_path", www_path);

    log_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
        "rosout", 10, std::bind(&Haru25_ros2 ::log_cb, this, _1));
    timer_ = this->create_wall_timer(10ms, std::bind(&Haru25_ros2::timer_cb, this));

    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    pos_sub_ = this->create_subscription<haru25_msgs::msg::Pos>(
        "pos", 10, std::bind(&Haru25_ros2 ::pos_cb, this, _1));

    joy_.buttons.resize(12);
    joy_.axes.resize(8);
}

void Haru25_ros2::timer_cb() {
    std::lock_guard<std::mutex> lock(mtx_);
    // webから来たデータを表示
    // for(int i=0;i<WEB_DATA_LEN;++i){
    //     std::cout << web_data_[i] << ", ";
    // }
    // std::cout << std::endl;
    if(web_receive_){
        joy_.header.stamp = this->get_clock()->now();
        for(int i=0;i<8;++i){
            if(buttommap[i] >= 0)
                joy_.buttons[buttommap[i]] = bitRead(web_data_[0], i);
            if(buttommap[i+8] >= 0)
                joy_.buttons[buttommap[i+8]] = bitRead(web_data_[1], i);
        }
        // LX LY RX RY
        int16_t axes;
        for(int i=0;i<4;++i){
            axes = (int16_t)(web_data_[2 + i*2 + 1] << 8 | web_data_[2 + i*2]);
            joy_.axes[axesmap[i]] = -(float)axes/2048.f;
        }
        joy_.axes[2] = bitRead(web_data_[0], LT) ? -1 : 1;
        joy_.axes[5] = bitRead(web_data_[0], RT) ? -1 : 1;
        joy_.axes[6] = bitRead(web_data_[1], LEFT) ? 1 : (bitRead(web_data_[1], RIGHT) ? -1 : 0);
        joy_.axes[7] = bitRead(web_data_[1], UP) ? 1 : (bitRead(web_data_[1], DOWN) ? -1 : 0);

        joy_pub_->publish(joy_);
    }
}
void Haru25_ros2::log_cb(const rcl_interfaces::msg::Log::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    const int c_len = msg->msg.size();
    if((ros2_data_str_len_ + c_len + 1) <= ROS2_LOG_LEN){
        std::char_traits<char>::copy(ros2_data_+ROS2_POS_LEN+ros2_data_str_len_, msg->msg.c_str(), c_len);
        ros2_data_[ROS2_POS_LEN+ros2_data_str_len_+c_len] = '\n';
        ros2_data_str_len_ += (c_len + 1);
    }
}

void Haru25_ros2::pos_cb(const haru25_msgs::msg::Pos::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    float data[6];
    data[0] = msg->x;
    data[1] = msg->y;
    data[2] = msg->yaw;
    data[3] = msg->vx;
    data[4] = msg->vy;
    data[5] = msg->vyaw;
    memcpy(ros2_data_, (char*)data, ROS2_POS_LEN);
}
// buffer：webから来るデータ、size：webから来るデータの数
boost::asio::mutable_buffer Haru25_ros2::getBuffer(boost::beast::flat_buffer buffer, std::size_t size){
    // これ以降は排他処理
    std::lock_guard<std::mutex> lock(mtx_);
    // std::cout << size << std::endl;
    //Webから来るデータを格納
    if(size == WEB_DATA_LEN){
        auto data = boost::asio::buffer_cast<uint8_t*>(buffer.data());//型変換
        memcpy(web_data_, data, WEB_DATA_LEN);//コピー
        web_receive_ = true;
    }
    int str_len = ros2_data_str_len_;
    ros2_data_str_len_ = 0;
    // ROS2から来るデータを返す
    return boost::asio::buffer(ros2_data_, ROS2_POS_LEN + str_len);
}