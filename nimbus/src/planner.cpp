#include "nimbus/planner.h"


/***************************
 *
 * QuadPosition Class
 *
 ***************************/


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/msg/waypoint.hpp>

// Action Interfaces
#include "router_interfaces/action/waypoint_move.hpp"
#include "router_interfaces/action/waypoint_sequence.hpp"

// Standard Libraries
#include <cmath>
#include <mutex>
#include <vector>
#include <queue>
#include <set>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <thread>
#include <iostream>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// Constants
#define DEBUG_PLANNER false
#define EARTH_RADIUS_M 6371000.0
#define INF std::numeric_limits<double>::infinity()

// MAVLink Commands
#define MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION 5001
#define MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION 5002

// ----------------------------------------------------------------------------
// Helper Structs & Classes
// ----------------------------------------------------------------------------

struct WaypointPosition {
    double latitude;
    double longitude;
    double relative_altitude;
};

struct Point2D {
    double x;
    double y;
};

// --- Coordinate Math Helpers (Replaces UTM) ---

// Converts Lat/Lon to Meters (Local Tangent Plane) relative to an origin
Point2D latlon_to_meters(double lat, double lon, double origin_lat, double origin_lon) {
    double d_lat = (lat - origin_lat) * M_PI / 180.0;
    double d_lon = (lon - origin_lon) * M_PI / 180.0;

    // Y is North (d_lat)
    // X is East (d_lon * cos(lat))
    double r_lat = origin_lat * M_PI / 180.0;

    double x = EARTH_RADIUS_M * d_lon * std::cos(r_lat);
    double y = EARTH_RADIUS_M * d_lat;

    return {x, y};
}

// Haversine Distance
double get_distance_meters(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return EARTH_RADIUS_M * c;
}

// --- Thread-Safe Data Classes ---

class PX4Mission {
public:
    PX4Mission() : has_mission_(false) {}

    void set_mission(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_mission_ = *msg;
        has_mission_ = true;
    }

    bool get_waypoint(size_t index, WaypointPosition& out_wp) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_mission_ || index >= latest_mission_.waypoints.size()) {
            return false;
        }
        const auto& wp = latest_mission_.waypoints[index];
        out_wp = {wp.x_lat, wp.y_long, wp.z_alt};
        return true;
    }

    std::vector<mavros_msgs::msg::Waypoint> get_all_waypoints() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_mission_) return {};
        return latest_mission_.waypoints;
    }

    bool has_mission() {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_mission_;
    }

    bool valid_waypoint(int index) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_mission_) return false;
        return (index >= 0 && index < (int)latest_mission_.waypoints.size());
    }

private:
    std::mutex mutex_;
    mavros_msgs::msg::WaypointList latest_mission_;
    bool has_mission_;
};

class PX4Geofence {
public:
    PX4Geofence() : has_fence_(false) {}

    void set_fence(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_fence_ = *msg;
        has_fence_ = true;
    }

    std::vector<mavros_msgs::msg::Waypoint> get_fence_points() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_fence_) return {};
        return latest_fence_.waypoints;
    }

    bool has_fence() {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_fence_;
    }

private:
    std::mutex mutex_;
    mavros_msgs::msg::WaypointList latest_fence_;
    bool has_fence_;
};

class QuadPosition {
public:
    QuadPosition() : has_position_(false) {}

    void set_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_position_ = *msg;
        has_position_ = true;
    }

    bool get_position(sensor_msgs::msg::NavSatFix& out_pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_position_) return false;
        out_pos = latest_position_;
        return true;
    }

private:
    std::mutex mutex_;
    sensor_msgs::msg::NavSatFix latest_position_;
    bool has_position_;
};

// ----------------------------------------------------------------------------
// Nimbus Planner Node
// ----------------------------------------------------------------------------

class NimbusPlanner : public rclcpp::Node {
public:
    using WaypointMove = router_interfaces::action::WaypointMove;
    using WaypointSequence = router_interfaces::action::WaypointSequence;
    using GoalHandleWaypointMove = rclcpp_action::ServerGoalHandle<WaypointMove>;
    using GoalHandleWaypointSequence = rclcpp_action::ClientGoalHandle<WaypointSequence>;

    NimbusPlanner() : Node("nimbus_planner") {

        // Callback group for concurrent Action Server/Client execution
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // QoS
        rclcpp::QoS best_effort(10);
        best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // Subscribers
        mission_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
            "/mavros/mission/waypoints", best_effort,
            std::bind(&NimbusPlanner::mission_callback, this, std::placeholders::_1));

        geofence_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
            "/mavros/geofence/fences", best_effort,
            std::bind(&NimbusPlanner::geofence_callback, this, std::placeholders::_1));

        position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", best_effort,
            std::bind(&NimbusPlanner::position_callback, this, std::placeholders::_1));

        // Action Server (Server)
        action_server_ = rclcpp_action::create_server<WaypointMove>(
            this,
            "planner/move_to_waypoint",
            std::bind(&NimbusPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NimbusPlanner::handle_cancel, this, std::placeholders::_1),
            std::bind(&NimbusPlanner::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            cb_group_
        );

        // Action Client (Client)
        navigator_client_ = rclcpp_action::create_client<WaypointSequence>(
            this, "/nimbus/navigator/set_route", cb_group_);

        RCLCPP_INFO(this->get_logger(), "Nimbus Planner initialized.");
    }

    // --- Public Interface for Main Loop ---
    bool has_data() {
        return px4_mission_.has_mission() && px4_geofence_.has_fence();
    }

    void build_graph() {
        RCLCPP_INFO(this->get_logger(), "Building waypoint graph...");

        auto mission_wps = px4_mission_.get_all_waypoints();
        auto fence_wps = px4_geofence_.get_fence_points();

        if (mission_wps.empty()) {
            RCLCPP_WARN(this->get_logger(), "No mission waypoints to build graph.");
            return;
        }

        // 1. Establish Coordinate System (Origin = Waypoint 0)
        double origin_lat = mission_wps[0].x_lat;
        double origin_lon = mission_wps[0].y_long;

        // 2. Convert Mission Nodes to Local Meters
        std::vector<Point2D> nodes_local;
        for (const auto& wp : mission_wps) {
            nodes_local.push_back(latlon_to_meters(wp.x_lat, wp.y_long, origin_lat, origin_lon));
        }

        // 3. Parse Geofences & Calculate Map Bounds
        double min_x = INF, max_x = -INF, min_y = INF, max_y = -INF;

        // Include nodes in bounds
        for(const auto& p : nodes_local) {
            if(p.x < min_x)
            	min_x = p.x;
            if(p.x > max_x)
            	max_x = p.x;
            if(p.y < min_y)
            	min_y = p.y;
            if(p.y > max_y)
            	max_y = p.y;
        }

        // Parse Fences
        struct Poly {
            std::vector<Point2D> points;
            int type; // 0 = exclusion, 1 = inclusion
        };
        std::vector<Poly> polygons;
        std::vector<Point2D> current_poly_pts;
        int current_type = 0; // default exclusion

        for (const auto& wp : fence_wps) {
            Point2D p = latlon_to_meters(wp.x_lat, wp.y_long, origin_lat, origin_lon);

            // Update bounds
            if(p.x < min_x)
            	min_x = p.x;
            if(p.x > max_x)
            	max_x = p.x;
            if(p.y < min_y)
            	min_y = p.y;
            if(p.y > max_y)
            	max_y = p.y;

            if (wp.command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
                if (current_type == 1 && !current_poly_pts.empty()) {
                    polygons.push_back({current_poly_pts, 1});
                    current_poly_pts.clear();
                }
                current_type = 0;
                current_poly_pts.push_back(p);
            } else if (wp.command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION) {
                if (current_type == 0 && !current_poly_pts.empty()) {
                    polygons.push_back({current_poly_pts, 0});
                    current_poly_pts.clear();
                }
                current_type = 1;
                current_poly_pts.push_back(p);
            } else {
                current_poly_pts.push_back(p);
            }
        }
        if (!current_poly_pts.empty()) {
            polygons.push_back({current_poly_pts, current_type});
        }

        // 4. Create OpenCV Map
        double width_m = max_x - min_x + (2 * MAP_BUFFER);
        double height_m = max_y - min_y + (2 * MAP_BUFFER);

        int img_w = std::ceil(width_m * MAP_RESOLUTION);
        int img_h = std::ceil(height_m * MAP_RESOLUTION);

        double offset_x = -min_x + MAP_BUFFER;
        double offset_y = -min_y + MAP_BUFFER;

        // Coord Transform to Pixel
        auto to_pix = [&](Point2D p) -> cv::Point {
            int px = (p.x + offset_x) * MAP_RESOLUTION;
            int py = img_h - ((p.y + offset_y) * MAP_RESOLUTION); // Flip Y
            return cv::Point(px, py);
        };

        // Initialize Map (0 = Safe, 255 = Obstacle)
        cv::Mat obst_map = cv::Mat::zeros(img_h, img_w, CV_8UC1);

        for (const auto& poly : polygons) {
            if (poly.type == 0) { // Exclusion
                std::vector<cv::Point> pts_pix;
                for(auto& p : poly.points) pts_pix.push_back(to_pix(p));

                std::vector<std::vector<cv::Point>> pts_wrapper = {pts_pix};
                cv::fillPoly(obst_map, pts_wrapper, cv::Scalar(255));
            }
        }

        // Dilation
        int kernel_size = OBSTACLE_DILATION * MAP_RESOLUTION;
        if (kernel_size > 0) {
             cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                cv::Size(2*kernel_size+1, 2*kernel_size+1));
             cv::dilate(obst_map, obst_map, element);
        }

        // 5. Build Graph
        graph_.clear();
        int num_edges = 0;

        for (size_t i = 0; i < nodes_local.size(); ++i) {
            for (size_t j = i + 1; j < nodes_local.size(); ++j) {
                // Distance check
                double d_x = nodes_local[i].x - nodes_local[j].x;
                double d_y = nodes_local[i].y - nodes_local[j].y;
                double dist = std::sqrt(d_x*d_x + d_y*d_y);

                if (dist > MAX_EDGE_LENGTH) continue;

                // Collision Check
                cv::Point p1 = to_pix(nodes_local[i]);
                cv::Point p2 = to_pix(nodes_local[j]);

                // Create a line mask for this edge
                cv::LineIterator it(obst_map, p1, p2, 8);
                bool collision = false;
                for(int k = 0; k < it.count; k++, ++it) {
                    if (obst_map.at<uchar>(it.pos()) > 0) { // 255 = Obstacle
                        collision = true;
                        break;
                    }
                }

                if (!collision) {
                    graph_[i][j] = dist;
                    graph_[j][i] = dist;
                    num_edges++;
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Graph built with %zu nodes and %d edges.", nodes_local.size(), num_edges);
    }

private:
    // Internal Data
    PX4Mission px4_mission_;
    PX4Geofence px4_geofence_;
    QuadPosition quad_position_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mission_sub_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr geofence_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_sub_;

    rclcpp_action::Server<WaypointMove>::SharedPtr action_server_;
    rclcpp_action::Client<WaypointSequence>::SharedPtr navigator_client_;

    // Graph: ID -> (Neighbor ID -> Weight)
    std::unordered_map<int, std::unordered_map<int, double>> graph_;

    // Config
    const double MAX_EDGE_LENGTH = 500.0;
    const double MAP_RESOLUTION = 10.0;
    const double MAP_BUFFER = 50.0;
    const int OBSTACLE_DILATION = 1;

    // --- Callbacks ---
    void mission_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received %zu mission waypoints.", msg->waypoints.size());
        px4_mission_.set_mission(msg);
    }

    void geofence_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received %zu geofence vertices.", msg->waypoints.size());
        px4_geofence_.set_fence(msg);
    }

    void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        quad_position_.set_position(msg);
    }

    // --- Action Server Logic ---
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const WaypointMove::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request for WP %d", goal->waypoint);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWaypointMove> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleWaypointMove> goal_handle) {
        // Execute in new thread
        std::thread{std::bind(&NimbusPlanner::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleWaypointMove> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing WaypointMove...");

        auto result = std::make_shared<WaypointMove::Result>();
        auto feedback = std::make_shared<WaypointMove::Feedback>();
        const auto goal = goal_handle->get_goal();

        // 1. Check Navigator Availability
        if (!navigator_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Navigator Action Server not available!");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 2. Validate Target
        if (!px4_mission_.valid_waypoint(goal->waypoint)) {
            RCLCPP_ERROR(this->get_logger(), "Target waypoint %d is invalid.", goal->waypoint);
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 3. Plan Route
        int start_idx = get_closest_waypoint();
        if (start_idx == -1) {
            // Fallback if GPS not ready
            RCLCPP_WARN(this->get_logger(), "Could not determine start index (no GPS?). Defaulting to 0.");
            start_idx = 0;
        }

        std::vector<int> route = calculate_route(start_idx, goal->waypoint);

        if (route.empty()) {
            RCLCPP_WARN(this->get_logger(), "No route found to waypoint %d", goal->waypoint);
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 4. Send to Navigator
        auto nav_goal = WaypointSequence::Goal();
        for(int wp : route) nav_goal.waypoints.push_back(wp);
        nav_goal.tolerance = goal->tolerance;

        RCLCPP_INFO(this->get_logger(), "Sending route size %zu to Navigator.", route.size());

        auto send_goal_options = rclcpp_action::Client<WaypointSequence>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [goal_handle, feedback](
                GoalHandleWaypointSequence::SharedPtr,
                const std::shared_ptr<const WaypointSequence::Feedback> nav_fb)
            {
                feedback->distance_to_go = nav_fb->progress;
                goal_handle->publish_feedback(feedback);
            };

        auto future_goal = navigator_client_->async_send_goal(nav_goal, send_goal_options);

        // Wait for acceptance
        if (future_goal.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
             RCLCPP_ERROR(this->get_logger(), "Navigator timed out accepting goal.");
             result->success = false;
             goal_handle->abort(result);
             return;
        }

        auto nav_goal_handle = future_goal.get();
        if (!nav_goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Navigator rejected goal.");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 5. Monitor Execution Loop
        auto result_future = navigator_client_->async_get_result(nav_goal_handle);

        while (rclcpp::ok()) {
            // Check for cancel
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Canceling Navigator goal...");
                navigator_client_->async_cancel_goal(nav_goal_handle);
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            // Check if finished
            if (result_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                auto wrapped_result = result_future.get();
                if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Route execution complete.");
                    result->success = true;
                    goal_handle->succeed(result);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Navigator execution failed.");
                    result->success = false;
                    goal_handle->abort(result);
                }
                return;
            }
        }
    }

    // --- Helpers ---

    int get_closest_waypoint() {
        sensor_msgs::msg::NavSatFix pos;
        if (!quad_position_.get_position(pos)) return -1;

        auto wps = px4_mission_.get_all_waypoints();
        if (wps.empty()) return -1;

        int best_idx = -1;
        double min_dist = INF;

        for (size_t i = 0; i < wps.size(); ++i) {
            double d = get_distance_meters(
                pos.latitude, pos.longitude,
                wps[i].x_lat, wps[i].y_long
            );
            if (d < min_dist) {
                min_dist = d;
                best_idx = static_cast<int>(i);
            }
        }
        return best_idx;
    }

    // --- A* Search ---
    struct AStarNode {
        int id;
        double f_score;
        bool operator>(const AStarNode& other) const { return f_score > other.f_score; }
    };

    std::vector<int> calculate_route(int start, int end) {
        if (graph_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Graph is empty!");
            return {};
        }

        // Heuristic Lambda
        WaypointPosition target_wp;
        if (!px4_mission_.get_waypoint(end, target_wp)) return {};

        auto heuristic = [&](int id) -> double {
            WaypointPosition wp;
            px4_mission_.get_waypoint(id, wp);
            return get_distance_meters(wp.latitude, wp.longitude, target_wp.latitude, target_wp.longitude);
        };

        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
        std::unordered_map<int, double> g_score;
        std::unordered_map<int, int> came_from;

        g_score[start] = 0.0;
        open_set.push({start, heuristic(start)});

        while(!open_set.empty()) {
            int current = open_set.top().id;
            open_set.pop();

            if (current == end) {
                // Reconstruct
                std::vector<int> path;
                while (current != start) {
                    path.push_back(current);
                    current = came_from[current];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (auto& edge : graph_[current]) {
                int neighbor = edge.first;
                double weight = edge.second;
                double tentative_g = g_score[current] + weight;

                if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g;
                    double f = tentative_g + heuristic(neighbor);
                    open_set.push({neighbor, f});
                }
            }
        }

        return {}; // No path
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NimbusPlanner>();

    // 1. Wait for Data (Throttle Logs)
    rclcpp::Rate rate(1.0);
    while (rclcpp::ok() && !node->has_data()) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
            "Waiting for valid PX4 Mission and Geofence...");
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // 2. Build Graph once data is present
    if (rclcpp::ok()) {
        node->build_graph();
    }

    // 3. Spin with MultiThreadedExecutor (required for Action Server + Client)
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
