#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "plan_path",
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
    
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);        
            
        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the map service. Exiting.");
                return;
            }
        
            RCLCPP_INFO(get_logger(), "Waiting for map service to appear...");
        }

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
        
        auto future_result = map_client_->async_send_request(
            request, 
            std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1)
        );

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Map received! Resolution: %.3f, Size: %dx%d", 
                    map_.info.resolution, map_.info.width, map_.info.height);
        
        dilateMap(); 
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to get map!");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, 
                            std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    
    RCLCPP_INFO(get_logger(), "Received path planning request.");
    
    // Clear old path
    path_.poses.clear();
    path_.header.stamp = this->now();
    path_.header.frame_id = map_.header.frame_id;

    // Create a new start pose based on the robot's actual position
    geometry_msgs::msg::PoseStamped actual_start;
    actual_start.header = path_.header;

    try {
        // tf2::TimePointZero means we want the latest available transform
        geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero);

        // Populate the actual_start pose with the translation and rotation from TF
        actual_start.pose.position.x = transformStamped.transform.translation.x;
        actual_start.pose.position.y = transformStamped.transform.translation.y;
        actual_start.pose.position.z = transformStamped.transform.translation.z;
        actual_start.pose.orientation = transformStamped.transform.rotation;
        
        RCLCPP_INFO(get_logger(), "Start position obtained from TF (Robot current pose).");

    } catch (const tf2::TransformException & ex) {
        // If TF fails, fallback to the requested start point
        RCLCPP_WARN(get_logger(), "Could not transform 'map' to 'base_link': %s. Falling back to requested start.", ex.what());
        actual_start = request->start; 
    }

    aStar(actual_start, request->goal);
    
    smoothPath();

    path_pub_->publish(path_);
    
    response->plan = path_;
}

void PlanningNode::dilateMap() {
    // Check if the map has been loaded successfully
    if (map_.data.empty()) {
        RCLCPP_WARN(get_logger(), "Map is empty. Cannot dilate.");
        return;
    }

    RCLCPP_INFO(get_logger(), "Performing binary map dilation...");

    // We read from the original map_ and write to dilatedMap to prevent a cascading effect
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    int width = map_.info.width;
    int height = map_.info.height;
    
    int dilation_radius = 10;

    // Loop through all cells in the grid
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            
            // Calculate 1D array index from the 2D coordinates
            int idx = y * width + x;
            
            // Check if the current cell is considered an obstacle
            if (map_.data[idx] > 50) {
                
                // Inflate the obstacle using a square shape (binary dilation)
                for (int dy = -dilation_radius; dy <= dilation_radius; ++dy) {
                    for (int dx = -dilation_radius; dx <= dilation_radius; ++dx) {
                        
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        // Check map boundaries 
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int n_idx = ny * width + nx;
                            dilatedMap.data[n_idx] = 100; 
                        }
                    }
                }
            }
        }
    }
    
    // Replace the node's original map with the fully dilated version
    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Map dilation completed.");
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    double res = map_.info.resolution;
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    int width = map_.info.width;
    int height = map_.info.height;

    int start_x = (start.pose.position.x - origin_x) / res;
    int start_y = (start.pose.position.y - origin_y) / res;
    
    int goal_x = (goal.pose.position.x - origin_x) / res;
    int goal_y = (goal.pose.position.y - origin_y) / res;

    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
        goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
        RCLCPP_ERROR(get_logger(), "Start or end of the path are out of bounds!");
        return;
    }

    RCLCPP_INFO(get_logger(), "Planning path from [%d, %d] to [%d, %d]", start_x, start_y, goal_x, goal_y);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(width * height, false);
    
    std::vector<std::shared_ptr<Cell>> allCells(width * height, nullptr);

    auto startCell = std::make_shared<Cell>(start_x, start_y);
    allCells[start_y * width + start_x] = startCell;
    openList.push_back(startCell);

    std::shared_ptr<Cell> goalCell = nullptr;

    while (!openList.empty() && rclcpp::ok()) {
        auto lowest_it = openList.begin();
        for (auto it = openList.begin(); it != openList.end(); ++it) {
            if ((*it)->f < (*lowest_it)->f) {
                lowest_it = it;
            }
        }

        std::shared_ptr<Cell> current = *lowest_it;
        openList.erase(lowest_it);

        if (current->x == goal_x && current->y == goal_y) {
            goalCell = current;
            break; 
        }

        closedList[current->y * width + current->x] = true;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = current->x + dx;
                int ny = current->y + dy;
                int nIdx = ny * width + nx;


                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                int cost = map_.data[nIdx];
                if (cost > 50 || cost < 0) continue;

                if (closedList[nIdx]) continue;

                double step_cost = (dx == 0 || dy == 0) ? 1.0 : 1.414;
                double tentative_g = current->g + step_cost;

                std::shared_ptr<Cell> neighbor = allCells[nIdx];

                if (neighbor == nullptr) {
                    neighbor = std::make_shared<Cell>(nx, ny);
                    allCells[nIdx] = neighbor;
                    
                    neighbor->g = tentative_g;
                    neighbor->h = std::sqrt(std::pow(nx - goal_x, 2) + std::pow(ny - goal_y, 2));
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;
                    
                    openList.push_back(neighbor);
                } 
                else if (tentative_g < neighbor->g) {
                    neighbor->g = tentative_g;
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;
                }
            }
        }
    }

    if (goalCell != nullptr) {
        RCLCPP_INFO(get_logger(), "Path found! Reconstructing...");
        std::shared_ptr<Cell> curr = goalCell;
        
        while (curr != nullptr) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_.header;

            pose.pose.position.x = curr->x * res + origin_x + (res / 2.0);
            pose.pose.position.y = curr->y * res + origin_y + (res / 2.0);
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path_.poses.push_back(pose);
            curr = curr->parent;
        }

        std::reverse(path_.poses.begin(), path_.poses.end());
        RCLCPP_INFO(get_logger(), "Path has %zu waypoints.", path_.poses.size());
    } else {
        RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    }
}

void PlanningNode::smoothPath() {
    if (path_.poses.size() < 3) {
        RCLCPP_WARN(get_logger(), "Path is too short to be smoothed.");
        return;
    }

    RCLCPP_INFO(get_logger(), "Smoothing path...");

    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    
    double weight_data = 0.5;   // How strongly points are pulled towards original path
    double weight_smooth = 0.1; // How strongly points are pulled towards neighbors (smoothness)
    double tolerance = 0.0001;  // Threshold to stop iterations when changes are extremely small
    int max_iterations = 1000;  // Safeguard against infinite loops
    
    double change = tolerance;
    int iterations = 0;

    while (change >= tolerance && iterations < max_iterations) {
        change = 0.0;
        
        for (size_t i = 1; i < path_.poses.size() - 1; ++i) {
            double x_orig = path_.poses[i].pose.position.x;
            double y_orig = path_.poses[i].pose.position.y;
            
            double x_curr = newPath[i].pose.position.x;
            double y_curr = newPath[i].pose.position.y;
            
            double x_prev = newPath[i - 1].pose.position.x;
            double y_prev = newPath[i - 1].pose.position.y;
            double x_next = newPath[i + 1].pose.position.x;
            double y_next = newPath[i + 1].pose.position.y;

            double old_x = x_curr;
            newPath[i].pose.position.x += weight_data * (x_orig - x_curr) + 
                                          weight_smooth * (x_next + x_prev - 2.0 * x_curr);
            change += std::abs(old_x - newPath[i].pose.position.x);

            double old_y = y_curr;
            newPath[i].pose.position.y += weight_data * (y_orig - y_curr) + 
                                          weight_smooth * (y_next + y_prev - 2.0 * y_curr);
            change += std::abs(old_y - newPath[i].pose.position.y);
        }
        iterations++;
    }

    path_.poses = newPath;
    
    RCLCPP_INFO(get_logger(), "Path smoothed successfully in %d iterations.", iterations);
}

Cell::Cell(int c, int r) {
    x = c;
    y = r;
    
    // g = cena od startu, h = heuristika, f = celková cena
    g = 0.0;
    h = 0.0;
    f = 0.0;
    
    parent = nullptr; 
}
