/*
 * navigator.h
 *
 * Created by:	Jonathan Diller
 * On: 			Feb 18, 2026
 *
 * Description: This is the navigator node of the Nimbus autonomy stack.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "router_interfaces/action/waypoint_move.hpp"
#include "router_interfaces/action/waypoint_sequence.hpp"


#define DEBUG_NAV false
#define DEFAULT_ACCEPTANCE_RADIUS 2.0


class Navigator : public rclcpp::Node {
public:
	// Define types for easier reference
	using WaypointMove = router_interfaces::action::WaypointMove;
	using WaypointSequence = router_interfaces::action::WaypointSequence;
	using GoalHandleWaypointMove = rclcpp_action::ClientGoalHandle<WaypointMove>;
	using GoalHandleWaypointSequence = rclcpp_action::ServerGoalHandle<WaypointSequence>;

	Navigator();
	virtual ~Navigator() = default;

private:
	double acceptance_radius_;
	// Track when we have completed a waypoint
	bool reached_waypoint_;
	double dist_to_waypoint_;
	double dist_total_;

	// Handle needed to cancel waypoint actions
	GoalHandleWaypointMove::SharedPtr current_client_goal_handle_;

	// Action Client
	rclcpp_action::Client<WaypointMove>::SharedPtr wp_action_client_;
	// Action Server
	rclcpp_action::Server<WaypointSequence>::SharedPtr move_action_server_;

	// Request Waypoint action from pilot
	void send_wp_goal(int waypoint, float tolerance);

	// Waypoint request response
	void wp_response_callback(const GoalHandleWaypointMove::SharedPtr & goal_handle);

	// Waypoint request feedback
	void wp_feedback_callback(GoalHandleWaypointMove::SharedPtr,
			const std::shared_ptr<const WaypointMove::Feedback> feedback_msg);

	// Waypoint request final result
	void wp_result_callback(const GoalHandleWaypointMove::WrappedResult & result);

	// Position goal validation
	rclcpp_action::GoalResponse position_goal_callback(
			const rclcpp_action::GoalUUID & uuid,
			std::shared_ptr<const WaypointSequence::Goal> goal);

	// Position cancel handling
	rclcpp_action::CancelResponse position_cancel_callback(
			const std::shared_ptr<GoalHandleWaypointSequence> goal_handle);

	// Position accepted (Starts execution thread)
	void position_accepted_callback(
			const std::shared_ptr<GoalHandleWaypointSequence> goal_handle);

	// Main position-move execution
	void position_execute_callback(
			const std::shared_ptr<GoalHandleWaypointSequence> goal_handle);
};
