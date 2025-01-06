#include "raspberry_gpio/multi_gpio_control_node.hpp"

MultiGPIOControlNode::MultiGPIOControlNode() : Node("multi_gpio_control_node")
{
    // Declare parameters
    this->declare_parameter<std::vector<int>>("pins", {26});        // List of GPIO pins
    this->declare_parameter<std::vector<int>>("modes", {0});        // Modes for each pin (0: OUTPUT, 1: INPUT, 2: PWM, 3: GPIO_CLOCK)
    this->declare_parameter<std::vector<int>>("pulls", {0});        // Pull modes (0: DOWN, 1: UP, 2: OFF)
    this->declare_parameter<int>("pub_rate", 500);                  // Publish rate for input pins

    // Get parameters and explicitly convert types
    auto pins_long = this->get_parameter("pins").as_integer_array();
    auto modes_long = this->get_parameter("modes").as_integer_array();
    auto pulls_long = this->get_parameter("pulls").as_integer_array();

    pins_ = std::vector<int>(pins_long.begin(), pins_long.end());
    modes_ = std::vector<int>(modes_long.begin(), modes_long.end());
    pulls_ = std::vector<int>(pulls_long.begin(), pulls_long.end());

    pub_rate_ = this->get_parameter("pub_rate").as_int();

    wiringPiSetupGpio();

    // Configure each pin
    for (size_t i = 0; i < pins_.size(); ++i) {
        int pin = pins_[i];
        int mode = (i < modes_.size()) ? modes_[i] : 0;  // Default to OUTPUT if not specified
        int pull = (i < pulls_.size()) ? pulls_[i] : 0;  // Default to PUD_DOWN if not specified

        if (mode == 0) {
            // Configure as OUTPUT
            pinMode(pin, OUTPUT);
            RCLCPP_INFO(this->get_logger(), "Pin %d configured as OUTPUT", pin);

            // Subscriber for output control
            auto topic_name = "/gpio/output_control_pin" + std::to_string(pin);
            auto sub = this->create_subscription<std_msgs::msg::Bool>(
                topic_name, 10,
                [this, pin](const std_msgs::msg::Bool::SharedPtr msg) {
                    digitalWrite(pin, msg->data ? HIGH : LOW);
                    RCLCPP_INFO(this->get_logger(), "Set Pin %d to %d", pin, msg->data);
                });

            output_subs_[pin] = sub;
        } else if (mode == 1) {
            // Configure as INPUT
            pinMode(pin, INPUT);
            if (pull == 1) {
                pullUpDnControl(pin, PUD_UP);
                RCLCPP_INFO(this->get_logger(), "Pin %d configured as INPUT with Pull-Up", pin);
            } else if (pull == 0) {
                pullUpDnControl(pin, PUD_DOWN);
                RCLCPP_INFO(this->get_logger(), "Pin %d configured as INPUT with Pull-Down", pin);
            } else {
                pullUpDnControl(pin, PUD_OFF);
                RCLCPP_INFO(this->get_logger(), "Pin %d configured as INPUT with No Pull", pin);
            }

            // Publisher for input state
            auto topic_name = "/gpio/input_state_pin" + std::to_string(pin);
            auto pub = this->create_publisher<std_msgs::msg::Bool>(topic_name, 10);
            input_pubs_[pin] = pub;
        } else if (mode == 2) {
            // Configure as PWM_OUTPUT
            pinMode(pin, PWM_OUTPUT);
            RCLCPP_INFO(this->get_logger(), "Pin %d configured as PWM OUTPUT", pin);
        } else if (mode == 3) {
            // Configure as GPIO_CLOCK
            pinMode(pin, GPIO_CLOCK);
            RCLCPP_INFO(this->get_logger(), "Pin %d configured as GPIO CLOCK", pin);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode %d for Pin %d", mode, pin);
        }
    }

    // Timer for publishing input states
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(pub_rate_),
        std::bind(&MultiGPIOControlNode::publishInputStates, this));
}


void MultiGPIOControlNode::publishInputStates()
{
    for (const auto &[pin, pub] : input_pubs_) {
        std_msgs::msg::Bool msg;
        msg.data = digitalRead(pin);
        pub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published state of Pin %d: %d", pin, msg.data);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiGPIOControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}