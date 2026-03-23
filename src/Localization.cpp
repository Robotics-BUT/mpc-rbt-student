#include "Localization.hpp"
#include "mpc_rbt_simulator/RobotConfig.hpp"
#include <cmath>

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    
    // Inicializace výchozí pozice a orientace na nulu
    odometry_.pose.pose.position.x = 0.0;
    odometry_.pose.pose.position.y = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    // Subscriber pro joint_states (čtení rychlosti kol)
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));

    // Publisher pro odometrii
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    // tf_broadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // Vypočet časového rozdílu (dt) od posledního zavolání
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Kontrola, zda nám přišly data pro obě kola (levé a pravé)
    if (msg.velocity.size() >= 2) {
        // Zde záleží na pořadí kol ve zprávě, obvykle index 0 = levé, index 1 = pravé
        updateOdometry(msg.velocity[0], msg.velocity[1], dt);
        publishOdometry();
        publishTransform();
    }
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // 1. Výpočet lineární a úhlové rychlosti robota z rychlosti kol
    // Předpokládáme, že RobotConfig.hpp obsahuje tyto konstanty
    double linear = (robot_config::WHEEL_RADIUS / 2.0) * (right_wheel_vel + left_wheel_vel);
    double angular = (robot_config::WHEEL_RADIUS / (2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS)) * (right_wheel_vel - left_wheel_vel);

    // 2. Získání aktuálního natočení (theta) z kvaternionu
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);
    
    // Normalizace úhlu
    theta = std::atan2(std::sin(theta), std::cos(theta));

    // 3. Eulerova integrace polohy (výpočet nové pozice)
    double x = odometry_.pose.pose.position.x;
    double y = odometry_.pose.pose.position.y;

    x += linear * std::cos(theta) * dt;
    y += linear * std::sin(theta) * dt;
    theta += angular * dt;

    // 4. Uložení nových hodnot zpět do zprávy odometry_
    odometry_.pose.pose.position.x = x;
    odometry_.pose.pose.position.y = y;

    // Převod nového úhlu theta zpět na kvaternion
    tf2::Quaternion q_new;
    q_new.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q_new);

    // Uložení aktuálních rychlostí
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;
}

void LocalizationNode::publishOdometry() {
    // Aktualizace časového razítka a odeslání zprávy
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    // Vytvoření a naplnění transformační zprávy
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odometry_.header.frame_id; // "map"
    t.child_frame_id = odometry_.child_frame_id;   // "base_link"

    // Zkopírování pozice z odometrie do transformace
    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;

    // Zkopírování rotace (kvaternionu) z odometrie do transformace
    t.transform.rotation = odometry_.pose.pose.orientation;

    // Odeslání transformace (TF)
    tf_broadcaster_->sendTransform(t);
}
