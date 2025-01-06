#ifndef GPIO_CONTROL_NODE_HPP
#define GPIO_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <wiringPi.h>

class GPIOControlNode : public rclcpp::Node
{
public:
    GPIOControlNode();

private:
    void publishInputState();
    void outputControlCallback(const std_msgs::msg::Bool::SharedPtr msg);

    int pin_;
    int mode_;
    int pull_;
    int pub_rate_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // GPIO_CONTROL_NODE_HPP