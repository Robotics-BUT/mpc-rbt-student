#ifndef KEYBOARD_CONTROL_HPP
#define KEYBOARD_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class KeyboardControlNode : public rclcpp::Node {
public:
    KeyboardControlNode();
    ~KeyboardControlNode();

private:
    void timerCallback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios old_termios_;
};

#endif // KEYBOARD_CONTROL_HPP
