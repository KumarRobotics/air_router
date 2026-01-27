/*
 * Command.h
 *
 * Created by:	Jonathan Diller
 * On: 			Nov 25, 2025
 *
 * Description: This is the pilot node for the Falcon4 autonomy stack. The working name for this stack is Nimbus,
 * because no one has given me a better name for it. If you don't like it, then you're welcome to change it yourself.
 */

#pragma once

#include <cmath>
#include <climits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/srv/waypoint_set_current.hpp>
#include <mavros_msgs/srv/waypoint_pull.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>

#include "router_interfaces/action/waypoint_move.hpp"


#define DEBUG_PILOT		false
// Earth radius in meters
#define EARTH_RADIUS_M	6371001.0
#define INF				1000000000.0


struct Waypoint_Position {
	double latitude;
	double longitude;
	double relative_altitude;
};


class QuadPosition {
public:
	QuadPosition() : has_position_(false) {}

	// Store latest position message
	void set_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

	// Getter:
	//  - copies stored latitude/longitude/altitude into *out_msg
	//  - returns true if at least one message has been stored
	bool get_position(sensor_msgs::msg::NavSatFix* out_msg);
	// Computes the distance from the quads current position to the given GSP position
	double distance_to(Waypoint_Position* wp_pos);

private:
	sensor_msgs::msg::NavSatFix latest_position_;
	sensor_msgs::msg::NavSatFix initial_position_;
	bool has_position_;
	std::mutex pos_mutex_;

	double degToRad(double degree);
};

class PX4Mission {
public:
	PX4Mission() : has_mission_(false) {}

	// Store latest mission message
	void set_mission(const mavros_msgs::msg::WaypointList::SharedPtr msg);

	// Copies the current mission into *out_msg, returns true if successful
	bool get_mission(mavros_msgs::msg::WaypointList* out_msg);
	// Copies the details of waypoint wp_id into pos, returns true if successful
	bool get_waypoint(int wp_id, Waypoint_Position* pos);
	// Checks to see if wp_id exists in the mission
	bool valid_waypoint(int wp_id);

private:
	mavros_msgs::msg::WaypointList latest_mission_;
	bool has_mission_;
	std::mutex mission_mutex_;
};



class Pilot : public rclcpp::Node {
public:
	using WaypointMove = router_interfaces::action::WaypointMove;
	using GoalHandleWaypointMove = rclcpp_action::ServerGoalHandle<WaypointMove>;

	Pilot();

private:
	// Publishers
	// Subscribers
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_sub_;
	rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mission_wp_sub_;
	// Services provided
	// Service clients
	rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr set_wp_client_;
	rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedPtr pull_wp_client_;

	// Action server
	rclcpp_action::Server<WaypointMove>::SharedPtr set_wp_action_server_;

	// Timers

	// Other variables
	QuadPosition quadPosition;
	PX4Mission px4Mission;

	// Set the target waypoint on the quad through MAVROS
	bool set_waypoint(uint16_t waypoint_index);

	// Callback for GPS position updates
	void position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
	// Mission waypoints callback
	void mission_wp_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg);
	// Mission waypoints callback
	void set_waypoint_callback(rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedFuture result);

	// Request mission waypoints from MAVROS/PX4
	/// This may take a while to complete! Should only be called once, unless the mission changes
	void pull_mission_waypoints();

	/// Action callbacks
	// WP Action accept goal callback -- Blindly accepts all goals
	rclcpp_action::GoalResponse handle_goal(
			const rclcpp_action::GoalUUID & uuid,
			std::shared_ptr<const WaypointMove::Goal> goal);
	// WP Action cancel callback -- acknowledges the cancel but does not stop the quad
	rclcpp_action::CancelResponse handle_cancel(
			const std::shared_ptr<GoalHandleWaypointMove> goal_handle);
	// WP Action execution callback -- starts a new thread and returns
	void handle_accepted(
			const std::shared_ptr<GoalHandleWaypointMove> goal_handle);
	// Worker function to monitor waypoint move. Returns when the quad reaches the waypoint or action is canceled.
	void execute(const std::shared_ptr<GoalHandleWaypointMove> goal_handle);
};

