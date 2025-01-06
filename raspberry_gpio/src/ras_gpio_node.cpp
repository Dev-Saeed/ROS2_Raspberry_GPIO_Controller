#include "raspberry_gpio/ras_gpio_node.hpp"


GPIOControlNode::GPIOControlNode()
    : Node("gpio_control_node")
{

        // Declare parameters
    this->declare_parameter<int>("pin", 26);
    this->declare_parameter<int>("mode", 0);
    this->declare_parameter<int>("pull", 0);
    this->declare_parameter<int>("pub_rate", 500);

    // Get parameters
    pin_ = this->get_parameter("pin").as_int();
    mode_ = this->get_parameter("mode").as_int();
    pull_ = this->get_parameter("pull").as_int();
    pub_rate_ = this->get_parameter("pub_rate").as_int();

    wiringPiSetupGpio();

    if (mode_ == 0) {
        pinMode(pin_, OUTPUT);
        RCLCPP_WARN(this->get_logger(), "GPIO Pin %d Configured as OUTPUT PIN", pin_);

        // Subscriber for output control
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "gpio/output_control", 10,
            std::bind(&GPIOControlNode::outputControlCallback, this, std::placeholders::_1));
        }

    else if (mode_ == 1) {
        pinMode(pin_, INPUT);

        if(pull_ == 1) { pullUpDnControl(pin_, PUD_UP);
        RCLCPP_WARN(this->get_logger(), "GPIO Pin %d Configured as INPUT PIN, Pull Up", pin_); }

        else if(pull_ == 0) { pullUpDnControl(pin_, PUD_DOWN);
        RCLCPP_WARN(this->get_logger(), "GPIO Pin %d Configured as INPUT PIN, Pull Down", pin_); }

        else { pullUpDnControl(pin_, PUD_OFF);
        RCLCPP_WARN(this->get_logger(), "GPIO Pin %d Configured as INPUT PIN, No Pull Up or Down", pin_); }

        // Publisher for input state
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("gpio/input_state", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(pub_rate_),
            std::bind(&GPIOControlNode::publishInputState, this));
    }

    else if (mode_ == 2) {
        pinMode(pin_, PWM_OUTPUT);
        RCLCPP_WARN(this->get_logger(), "GPIO Pin %d Configured as PWM PIN", pin_);
    }

    else if (mode_ == 3) {
        pinMode(pin_, GPIO_CLOCK);
        RCLCPP_WARN(this->get_logger(), "GPIO Pin %d Configured as GPIO CLOCK PIN", pin_);
    }

}

void GPIOControlNode::publishInputState()
{
    std_msgs::msg::Bool msg;
    msg.data = digitalRead(pin_); // Read the input pin state
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published input state: %d", msg.data);
}

void GPIOControlNode::outputControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received output control: %d", msg->data);
    digitalWrite(pin_, msg->data ? HIGH : LOW); // Control the output pin
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPIOControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
