/*
 * planner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Feb 16, 2026
 *
 * Description: This is the path planner node for the Nimbus autonomy stack.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_action/action_server.h>
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


#define DEBUG_PILOT		false

// Constants
#define DEBUG_PLANNER false
#define EARTH_RADIUS_M 6371000.0
#define INF std::numeric_limits<double>::infinity()



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

// Parse Fences
struct Poly {
	std::vector<Point2D> points;
	int type; // 0 = exclusion, 1 = inclusion
};

struct AStarNode {
    int id;
    double f_score;
    bool operator>(const AStarNode& other) const { return f_score > other.f_score; }
};

enum E_MavCMDPolygonType {
	INCLUSIVE = 5001,
	EXCLUSIVE = 5002
};



class PX4Mission {
public:
	PX4Mission() : has_mission_(false) {}

	void set_mission(const mavros_msgs::msg::WaypointList::SharedPtr msg);
	bool get_waypoint(size_t index, WaypointPosition& out_wp);
	// Fills waypoints vector with waypoints that we currently track. Returns false if we do not have a PX4 mission, true o.w.
	bool get_all_waypoints(std::vector<mavros_msgs::msg::Waypoint>* waypoints);
	bool has_mission();
	bool valid_waypoint(size_t index);

private:
	std::mutex ms_mutex_;
	mavros_msgs::msg::WaypointList latest_mission_;
	bool has_mission_;
};


class PX4Geofence {
public:
	PX4Geofence() : has_fence_(false) {}

	void set_fence(const mavros_msgs::msg::WaypointList::SharedPtr msg);
	// Fills vector with geofence waypoints that we currently track. Returns false if we do not have any geofence data, true o.w.
	bool get_fence_points(std::vector<mavros_msgs::msg::Waypoint>* fence_points);
	bool has_fence();

private:
	std::mutex gf_mutex_;
	mavros_msgs::msg::WaypointList latest_fence_;
	bool has_fence_;
};


class QuadPosition {
public:
	QuadPosition() : has_position_(false) {}

	void set_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	bool get_position(sensor_msgs::msg::NavSatFix& out_pos);

private:
	std::mutex pos_mutex_;
	sensor_msgs::msg::NavSatFix latest_position_;
	bool has_position_;
};


class NimbusPlanner : public rclcpp::Node {
public:
	using WaypointMove = router_interfaces::action::WaypointMove;
	using WaypointSequence = router_interfaces::action::WaypointSequence;
	using GoalHandleWaypointMove = rclcpp_action::ServerGoalHandle<WaypointMove>;
	using GoalHandleWaypointSequence = rclcpp_action::ClientGoalHandle<WaypointSequence>;

	NimbusPlanner();

private:
	// Internal Data
	PX4Mission px4_mission_;
	PX4Geofence px4_geofence_;
	QuadPosition quad_position_;

	// Subscribers
	rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mission_sub_;
	rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr geofence_sub_;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_sub_;

	// Action server/clients
	rclcpp::CallbackGroup::SharedPtr cb_group_;
	rclcpp_action::Server<WaypointMove>::SharedPtr action_server_;
	rclcpp_action::Client<WaypointSequence>::SharedPtr navigator_client_;

	// Startup timer
	rclcpp::TimerBase::SharedPtr startup_timer_;

	// Graph: ID -> (Neighbor ID -> Weight)
	std::unordered_map<int, std::unordered_map<int, double>> graph_;

	// Config
	const double MAX_EDGE_LENGTH = 500.0;
	const double MAP_RESOLUTION = 10.0;
	const double MAP_BUFFER = 50.0;
	const int OBSTACLE_DILATION = 1;

	// --- Graph building functions ---
	bool has_graph_data();
	// Build waypoint graph
	void build_graph();

	// --- Callbacks ---
	void startup_check_callback();
	void mission_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg);
	void geofence_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg);
	void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

	// --- Action Server Logic ---
	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const WaypointMove::Goal> goal);
	rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWaypointMove> goal_handle);
	void handle_accepted(const std::shared_ptr<GoalHandleWaypointMove> goal_handle);
	// Sub-process function for executing waypoint move
	void execute(const std::shared_ptr<GoalHandleWaypointMove> goal_handle);

	// --- Helpers ---
	int get_closest_waypoint();
	// Converts Lat/Lon to Meters (Local Tangent Plane) relative to an origin
	Point2D latlon_to_meters(double lat, double lon, double origin_lat, double origin_lon);
	// Haversine Distance
	double get_distance_meters(double lat1, double lon1, double lat2, double lon2);

	// --- A* Search ---
	bool calculate_route(int start, int end, std::vector<int>* route);
	double heuristic(int node_id, int goal_id);
	bool A_start(int start, int end, std::vector<int>* route);
};
