#include "haru25_hardware/PacketSerial.h"
#include "haru25_msgs/msg/can.hpp"
#include "haru25_msgs/msg/can_array.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class HaruHardware : public rclcpp::Node {
    rclcpp::Publisher<haru25_msgs::msg::CANArray>::SharedPtr can_pub_;
    rclcpp::Subscription<haru25_msgs::msg::CANArray>::SharedPtr can_sub_;

    PacketSerial packet_;
    uint8_t rx_data_[BUFFER_SIZE], tx_data_[BUFFER_SIZE];
    bool connect = false;
    int tx_len_ = 0;
    std::string port_;

   public:
    HaruHardware()
        : Node("haru25_hardware") {
        declare_parameter("port", "/dev/ttyACM0");
        port_ = this->get_parameter("port").as_string();
        can_pub_ = this->create_publisher<haru25_msgs::msg::CANArray>("can/rx", 10);
        can_sub_ = this->create_subscription<haru25_msgs::msg::CANArray>(
            "can/tx", 10, std::bind(&HaruHardware ::can_cb, this, _1));
    }

   private:
    void update() {
        int uint8_len, flt_len, rx_len, i;
        float* flt_addr;
        haru25_msgs::msg::CAN can;
        haru25_msgs::msg::CANArray can_array;

        try {
            // もしつながっていなかったら
            if (!connect) {
                // Teensyと接続する
                packet_.connect(port_.c_str());
                connect = true;
                RCLCPP_INFO(this->get_logger(), "connect");
            } else {
                // 送信
                packet_.send(tx_data_, tx_len_);
                tx_len_ = 0;
                // 受信
                rx_len = packet_.read(rx_data_);
                // CANに変換
                i = 0;
                while (i < rx_len) {
                    can.id = rx_data_[i];
                    uint8_len = rx_data_[i + 1];
                    flt_len = uint8_len / 4;
                    flt_addr = (float*)&rx_data_[i + 2];
                    can.data.resize(flt_len);
                    can.data.assign(flt_addr, flt_addr + flt_len);
                    i += (uint8_len + 2);
                    can_array.array.push_back(can);
                }
                can_pub_->publish(can_array);
            }
            // 接続できなかった
        } catch (...) {
            connect = false;
            RCLCPP_INFO(this->get_logger(), "no connect");
            // １秒待つ
            rclcpp::sleep_for(1000ms);
        }
    }
    void can_cb(const haru25_msgs::msg::CANArray::SharedPtr msg) {
        int flt_len, uint8_len;
        uint8_t* uint8_addr;
        bool sub_wheel = false;
        for (auto& can : msg->array) {
            flt_len = can.data.size();
            uint8_len = flt_len * 4;
            if (tx_len_ + uint8_len > BUFFER_SIZE)
                return;
            tx_data_[tx_len_] = can.id;
            tx_data_[tx_len_ + 1] = uint8_len;
            uint8_addr = (uint8_t*)can.data.data();
            std::copy(uint8_addr, uint8_addr + uint8_len, tx_data_ + tx_len_ + 2);
            tx_len_ += (uint8_len + 2);

            sub_wheel |= (can.id < 4);
        }
        if(sub_wheel)
            update();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HaruHardware>());
    rclcpp::shutdown();
    return 0;
}