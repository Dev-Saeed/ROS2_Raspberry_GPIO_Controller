// multi_gpio_control_node.hpp
#ifndef MULTI_GPIO_CONTROL_NODE_HPP
#define MULTI_GPIO_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <wiringPi.h>
#include <vector>
#include <unordered_map>
#include <string>

class MultiGPIOControlNode : public rclcpp::Node
{
public:
    MultiGPIOControlNode();

private:
    void publishInputStates();

    // Member variables
    std::vector<int> pins_;
    std::vector<int> modes_;
    std::vector<int> pulls_;
    int pub_rate_;

    std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> input_pubs_;
    std::unordered_map<int, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> output_subs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MULTI_GPIO_CONTROL_NODE_HPP