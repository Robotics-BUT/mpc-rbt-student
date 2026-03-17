#include "../include/KeyboardControl.hpp"

KeyboardControlNode::KeyboardControlNode() : Node("keyboard_control_node") {
    // Deklarace parametru pro rychlost (zisk plného počtu bodů)
    this->declare_parameter<double>("speed", 0.5);

    // Vytvoření publisheru a timeru
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&KeyboardControlNode::timerCallback, this)
    );

    // Nastavení terminálu pro neblokující čtení klávesnice
    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");
}

KeyboardControlNode::~KeyboardControlNode() {
    // Obnovení původního nastavení terminálu při ukončení
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    geometry_msgs::msg::Twist twist{};
    char c;
    
    // Vyčtení aktuální hodnoty parametru rychlosti
    double current_speed = this->get_parameter("speed").as_double();

    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

    if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\033') { // ESC sequence (šipky)
                char seq[2];
                if (read(STDIN_FILENO, &seq, 2) != 2) return;

                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A': twist.linear.x = current_speed;  break;  // Nahoru
                        case 'B': twist.linear.x = -current_speed; break;  // Dolů
                        case 'C': twist.angular.z = -current_speed; break; // Doprava
                        case 'D': twist.angular.z = current_speed;  break; // Doleva
                    }
                }
            }
            // Publikování zprávy při stisku klávesy
            twist_publisher_->publish(twist);
        }
    }
}
