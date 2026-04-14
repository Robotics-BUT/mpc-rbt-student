#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {
        // Subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
            
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/tiago_base/Hokuyo_URG_04LX_UG01", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

        // Publishers
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this, "go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Motion control node started.");
    }

void MotionControlNode::checkCollision() {
    if (laser_scan_.ranges.empty() || !goal_handle_ || !goal_handle_->is_active()) return;

    double threshold = 0.3;
    
    int center_idx = laser_scan_.ranges.size() / 2;
    int search_range = laser_scan_.ranges.size() / 3;

    for (int i = center_idx - search_range; i <= center_idx + search_range; ++i) {
        if (i >= 0 && i < (int)laser_scan_.ranges.size()) {
            float range = laser_scan_.ranges[i];
            
            if (!std::isnan(range) && range > laser_scan_.range_min && range < threshold) {
                RCLCPP_FATAL(get_logger(), "EMERGENCY STOP! Obstacle detected at %f meters.", range);
                
                geometry_msgs::msg::Twist stop;
                stop.linear.x = 0.0;
                stop.angular.z = 0.0;
                twist_publisher_->publish(stop);

                auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                goal_handle_->abort(result);
                break;
            }
        }
    }
}

void MotionControlNode::updateTwist() {
    if (path_.poses.empty() || !goal_handle_ || !goal_handle_->is_active()) return;

    double rx = current_pose_.pose.position.x;
    double ry = current_pose_.pose.position.y;
    
    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double lookahead_distance = 0.3;

    while (!path_.poses.empty()) {
        double dx = path_.poses.front().pose.position.x - rx;
        double dy = path_.poses.front().pose.position.y - ry;
        if (std::sqrt(dx*dx + dy*dy) < lookahead_distance) {
            path_.poses.erase(path_.poses.begin());
        } else {
            break;
        }
    }

    if (path_.poses.empty()) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        return;
    }

    geometry_msgs::msg::PoseStamped target_pt = path_.poses.front();

    double dx_world = target_pt.pose.position.x - rx;
    double dy_world = target_pt.pose.position.y - ry;
    
    double x_r = std::cos(-yaw) * dx_world - std::sin(-yaw) * dy_world;
    double y_r = std::sin(-yaw) * dx_world + std::cos(-yaw) * dy_world;

    double v_max = 0.3; 
    double angular_vel = 0.0;
    
    double alpha = std::atan2(y_r, x_r);

    if (std::abs(alpha) > 0.5) {
        v_max = 0.0;

        double Kp_turn = 1.5;
        angular_vel = Kp_turn * alpha;
    } else {
        double L_d = std::max(std::sqrt(x_r*x_r + y_r*y_r), 0.01);
        angular_vel = (2.0 * v_max * y_r) / (L_d * L_d);
    }

    double max_angular = 1.0; 
    if (angular_vel > max_angular) angular_vel = max_angular;
    if (angular_vel < -max_angular) angular_vel = -max_angular;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = v_max;
    twist.angular.z = angular_vel;
    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received goal request.");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    goal_handle_ = goal_handle;
    goal_pose_ = goal_handle->get_goal()->pose;

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal_pose_;

    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get();
    if (response && response->plan.poses.size() > 0) {
        RCLCPP_INFO(get_logger(), "Path received! Starting execution.");
        path_ = response->plan;
        
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to get path. Aborting goal.");
        if (goal_handle_) goal_handle_->abort(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
    }
}
void MotionControlNode::execute() {
    rclcpp::Rate loop_rate(10.0);

    while (rclcpp::ok()) {
        if (!goal_handle_ || !goal_handle_->is_active()) {
            return;
        }

        if (goal_handle_->is_canceling()) {
            RCLCPP_INFO(get_logger(), "Goal canceled by user.");
            
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->canceled(result);
            return;
        }

        double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
        double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
        double distance_to_goal = std::sqrt(dx*dx + dy*dy);

        auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
        feedback->distance_remaining = distance_to_goal;
        goal_handle_->publish_feedback(feedback);

        if (distance_to_goal < 0.1) {
            RCLCPP_INFO(get_logger(), "Goal reached successfully!");
            
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->succeed(result);
            return;
        }

        loop_rate.sleep();
    }
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.pose = msg.pose.pose;
    current_pose_.header = msg.header;

    checkCollision();
    updateTwist();
}
