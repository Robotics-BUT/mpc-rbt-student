#include <chrono>
#include <functional>

#include "KeyboardControl.hpp"

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode(): rclcpp::Node("keyboard_control_node") {

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardControlNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Keyboard Control node started.");
}

KeyboardControlNode::~KeyboardControlNode() {
}

void KeyboardControlNode::timerCallback() {
}
