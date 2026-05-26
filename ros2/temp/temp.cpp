#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class TempNode : public rclcpp::Node {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr str_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

   public:
    TempNode()
        : Node("temp") {
        str_pub_ = this->create_publisher<std_msgs::msg::String>("pub", 10);
        str_sub_ = this->create_subscription<std_msgs::msg::String>(
            "sub", 10, std::bind(&TempNode ::str_cb, this, _1));
        timer_ = this->create_wall_timer(500ms, std::bind(&TempNode::timer_cb, this));
    }

   private:
    void timer_cb() {
        RCLCPP_INFO(this->get_logger(), "run");
    }
    void str_cb(const std_msgs::msg::String::SharedPtr msg) {
        auto message = std_msgs::msg::String();
        message.data = "I heard " + msg->data;
        str_pub_->publish(message);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TempNode>());
    rclcpp::shutdown();
    return 0;
}