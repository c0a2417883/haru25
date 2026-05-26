#pragma once

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "haru25_msgs/msg/pos.hpp"
#include <boost/beast/core.hpp>

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

class Haru25_ros2 : public rclcpp::Node {
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
    rclcpp::Subscription<haru25_msgs::msg::Pos>::SharedPtr pos_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Joy joy_;

   public:
    Haru25_ros2();
    boost::asio::mutable_buffer getBuffer(boost::beast::flat_buffer, std::size_t);
    std::string www_path;

   private:
    void timer_cb();
    void log_cb(const rcl_interfaces::msg::Log::SharedPtr msg);
    void pos_cb(const haru25_msgs::msg::Pos::SharedPtr msg);
};