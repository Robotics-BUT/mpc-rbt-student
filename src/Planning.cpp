#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // NOVÉ: Pro waypointy trasy
#include <vector>                             // NOVÉ: Pro vektory (otevřený/zavřený list)
#include <cmath>                              // NOVÉ: Pro matematické funkce (vzdálenost)
#include <algorithm>                          // NOVÉ: Pro std::reverse

using namespace std::placeholders;

// NOVÉ: Pomocná struktura pro algoritmus A*
struct Cell {
    int x, y;          // Souřadnice v mřížce mapy
    double f, g, h;    // Ceny: g (od startu), h (k cíli - heuristika), f = g + h
    int parent_x, parent_y; // Pro zpětnou rekonstrukci trasy
};

class PlanningNode : public rclcpp::Node {
public:
    PlanningNode() : Node("planning_node") {
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
        
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "/plan_path",
            std::bind(&PlanningNode::planPathCallback, this, _1, _2)
        );

        // NOVÉ: Inicializace publisheru pro vizualizaci naplánované trasy
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(this->get_logger(), "Node Planning inicializován.");
        requestMap();
    }

private:
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_service_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // NOVÉ: Publisher
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;

    // --- NOVÉ: POMOCNÉ FUNKCE PRO A* ---

    // Převod reálných souřadnic (metry) na indexy v mapě
    bool worldToMap(double wx, double wy, int& mx, int& my) {
        if (!map_received_) return false;
        mx = std::round((wx - current_map_.info.origin.position.x) / current_map_.info.resolution);
        my = std::round((wy - current_map_.info.origin.position.y) / current_map_.info.resolution);
        
        // Kontrola, zda nejsme mimo mapu
        if (mx < 0 || mx >= (int)current_map_.info.width || my < 0 || my >= (int)current_map_.info.height) {
            return false;
        }
        return true;
    }

    // Převod indexů mapy zpět na reálné souřadnice
    void mapToWorld(int mx, int my, double& wx, double& wy) {
        wx = mx * current_map_.info.resolution + current_map_.info.origin.position.x;
        wy = my * current_map_.info.resolution + current_map_.info.origin.position.y;
    }

    // Heuristická funkce pro A* (Euklidovská vzdálenost)
    double calculateHValue(int x, int y, int goal_x, int goal_y) {
        return std::sqrt(std::pow(x - goal_x, 2) + std::pow(y - goal_y, 2));
    }

    // Převedení 2D souřadnic na 1D index (mapa je row-major)
    int getIndex(int x, int y) {
        return y * current_map_.info.width + x;
    }

    // Samotný A* algoritmus (Zatím vrací zjednodušenou přímou trasu pro test zapojení)
    std::vector<geometry_msgs::msg::PoseStamped> aStar(
        const geometry_msgs::msg::PoseStamped& start, 
        const geometry_msgs::msg::PoseStamped& goal) 
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        int start_x, start_y, goal_x, goal_y;

        // Převedeme světové souřadnice startu a cíle do mapy
        if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
            !worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
            RCLCPP_ERROR(this->get_logger(), "Start nebo cíl leží mimo mapu!");
            return path; 
        }

        RCLCPP_INFO(this->get_logger(), "A* plánuje v mapě z [%d, %d] do [%d, %d]", start_x, start_y, goal_x, goal_y);

        // TODO pro příště: Zde bude plná logika A* (open list, closed list, iterování sousedů)
        // Prozatím (abychom otestovali napojení publisheru), vygenerujeme "falešnou" cestu 
        // rovnou čárou, abychom viděli výsledek v RVizu.
        
        for (double t = 0.0; t <= 1.0; t += 0.1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
            pose.pose.position.y = start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
            path.push_back(pose);
        }

        return path;
    }

    // --- PŮVODNÍ FUNKCE (mírně upravené pro zapojení A*) ---

    void requestMap() {
        if (!map_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Služba mapy zatím není dostupná. Ujisti se, že běží map_server!");
        }
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        RCLCPP_INFO(this->get_logger(), "Odesílám požadavek na načtení mapy...");
        auto future = map_client_->async_send_request(request, std::bind(&PlanningNode::mapResponseCallback, this, _1));
    }

    void mapResponseCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
        auto response = future.get();
        current_map_ = response->map;
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Mapa úspěšně načtena! Rozlišení: %f, Šířka: %d, Výška: %d", 
                    current_map_.info.resolution, current_map_.info.width, current_map_.info.height);
    }

    void planPathCallback(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                          std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Přijat požadavek na plánování trasy z [%f, %f] do [%f, %f].",
                    request->start.pose.position.x, request->start.pose.position.y,
                    request->goal.pose.position.x, request->goal.pose.position.y);
        
        if (!map_received_) {
            RCLCPP_WARN(this->get_logger(), "Nemohu plánovat, mapa ještě nebyla načtena!");
            return;
        }

        // NOVÉ: Zavoláme náš A* algoritmus
        auto planned_poses = aStar(request->start, request->goal);

        // Naplníme odpověď služby
        response->plan.header.stamp = this->now();
        response->plan.header.frame_id = "map";
        response->plan.poses = planned_poses;

        // NOVÉ: Zveřejníme trasu na topic, aby se dala zobrazit v RVizu
        path_pub_->publish(response->plan);
        
        RCLCPP_INFO(this->get_logger(), "Trasa vygenerována a publikována (%zu bodů).", planned_poses.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
