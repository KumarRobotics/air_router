#include "nimbus/navigator.h"


/***************************
 *
 * Navigator Class
 *
 ***************************/

using namespace std::placeholders;

Navigator::Navigator() : Node("navigator") {
	// Declare parameters
	this->declare_parameter("acceptance_radius", DEFAULT_ACCEPTANCE_RADIUS);

	// Get the acceptance radius
	try {
		acceptance_radius_ = this->get_parameter("acceptance_radius").as_double();
	}
	catch (const rclcpp::ParameterTypeException & e) {
		RCLCPP_ERROR(this->get_logger(), "Acceptance radius type mismatch.");
		acceptance_radius_ = DEFAULT_ACCEPTANCE_RADIUS;
	}

	// Validate acceptance radius
	if(acceptance_radius_ < 1.0 || acceptance_radius_ > 20.0) {
		RCLCPP_INFO(this->get_logger(), "%s: Acceptance radius should be a float between 1 and 20", this->get_name());
		rclcpp::shutdown();
		return;
	}

	// Initialize state
	current_client_goal_handle_ = nullptr;
	reached_waypoint_ = true;
	dist_to_waypoint_ = 0.0;
	dist_total_ = -1.0;

	// Create action client
	wp_action_client_ = rclcpp_action::create_client<WaypointMove>(
		this,
		"/nimbus/pilot/set_waypoint"
	);

	// Create the action server
	move_action_server_ = rclcpp_action::create_server<WaypointSequence>(
		this,
		"navigator/set_route",
		std::bind(&Navigator::position_goal_callback, this, _1, _2),
		std::bind(&Navigator::position_cancel_callback, this, _1),
		std::bind(&Navigator::position_accepted_callback, this, _1)
	);

	RCLCPP_INFO(this->get_logger(), "Started %s:, AR: %.2f", this->get_name(), acceptance_radius_);
}

// --- Request Waypoint action from pilot --------------------------------
void Navigator::send_wp_goal(int waypoint, float tolerance) {
	// Wait for the action server to come online
	if(!wp_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
		RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
		return;
	}

	// Create goal message
	auto goal_msg = WaypointMove::Goal();
	goal_msg.waypoint = waypoint;
	goal_msg.tolerance = tolerance;

	// Goal tracking
	reached_waypoint_ = false;

	// Logging
	RCLCPP_INFO(this->get_logger(), "Sending pilot waypoint %d, tolerance = %.2f", waypoint, tolerance);

	// Send goal asynchronously
	auto send_goal_options = rclcpp_action::Client<WaypointMove>::SendGoalOptions();

	// Attach callbacks
	send_goal_options.goal_response_callback =
		std::bind(&Navigator::wp_response_callback, this, _1);

	send_goal_options.feedback_callback =
		std::bind(&Navigator::wp_feedback_callback, this, _1, _2);

	send_goal_options.result_callback =
		std::bind(&Navigator::wp_result_callback, this, _1);

	wp_action_client_->async_send_goal(goal_msg, send_goal_options);
}

// --- Waypoint request response -----------------------------------------
void Navigator::wp_response_callback(const GoalHandleWaypointMove::SharedPtr & goal_handle) {
	if(!goal_handle) {
		RCLCPP_WARN(this->get_logger(), "Waypoint rejected");
		reached_waypoint_ = true;
		return;
	}

	RCLCPP_INFO(this->get_logger(), "Waypoint accepted");

	// Store the action handle (used to cancel action, if needed)
	current_client_goal_handle_ = goal_handle;
}

// --- Waypoint request feedback -----------------------------------------
void Navigator::wp_feedback_callback(GoalHandleWaypointMove::SharedPtr, const std::shared_ptr<const WaypointMove::Feedback> feedback_msg) {
	dist_to_waypoint_ = feedback_msg->distance_to_go;

	if(dist_total_ < 0) {
		dist_total_ = feedback_msg->distance_to_go;
	}

	// Logging
	if(DEBUG_NAV) {
		RCLCPP_INFO(this->get_logger(), "Distance to go: %.2f m", feedback_msg->distance_to_go);
	}
}

// --- Waypoint request final result -------------------------------------
void Navigator::wp_result_callback(const GoalHandleWaypointMove::WrappedResult & result) {
	// Regardless of the result, mark that the action ended
	reached_waypoint_ = true;
	dist_total_ = -1;

	bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
	RCLCPP_INFO(this->get_logger(), "Waypoint action result: %s", success ? "True" : "False");
}

// --- Position goal callback ------------------------------------------
rclcpp_action::GoalResponse Navigator::position_goal_callback(const rclcpp_action::GoalUUID&, std::shared_ptr<const WaypointSequence::Goal> goal_request) {
	RCLCPP_INFO(this->get_logger(), "Received WaypointSequence request, tolerance: %.2f", goal_request->tolerance);
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// --- Position cancel handling ------------------------------------------
rclcpp_action::CancelResponse Navigator::position_cancel_callback(const std::shared_ptr<GoalHandleWaypointSequence>) {
	RCLCPP_INFO(this->get_logger(), "Received cancel request.");
	return rclcpp_action::CancelResponse::ACCEPT;
}

// --- Position accepted callback ----------------------------------------
void Navigator::position_accepted_callback(const std::shared_ptr<GoalHandleWaypointSequence> goal_handle) {
	// Execute action (follow route) in new thread
	std::thread{std::bind(&Navigator::position_execute_callback, this, _1), goal_handle}.detach();
}

// --- Main position-move execution --------------------------------------
void Navigator::position_execute_callback(const std::shared_ptr<GoalHandleWaypointSequence> goal_handle) {
	RCLCPP_INFO(this->get_logger(), "Executing WaypointSequence action...");

	const auto goal = goal_handle->get_goal();

	// Copy the route so we can modify (pop) it
	std::queue<int> quad_route;
	for(const int wp : goal->waypoints) {
		quad_route.push(wp);
	}
//	std::vector<int> quad_route = goal->waypoints;

	// Waypoint tolerance should be no less than navigator's acceptance radius
	double waypoint_tolerance = std::max((double)goal->tolerance, acceptance_radius_);

	// Verify that the sequence isn't empty...
	if(quad_route.empty()) {
		RCLCPP_INFO(this->get_logger(), "Received empty waypoint sequence!");
		auto result = std::make_shared<WaypointSequence::Result>();
		result->success = true;
		goal_handle->succeed(result);
		return;
	}

	// Print the goal info (Accessing last element safely)
	RCLCPP_INFO(
		this->get_logger(),
		"Received WaypointSequence w/ %zu stops, going to %d",
		quad_route.size(),
		quad_route.back()
	);

	// Track progress along route
	auto feedback_msg = std::make_shared<WaypointSequence::Feedback>();
	double work_complete = 0;
	double total_work = static_cast<double>(quad_route.size());

	// Send first waypoint
	send_wp_goal(quad_route.front(), waypoint_tolerance);
	quad_route.pop();

	rclcpp::Rate loop_rate(10); // Check status at 10Hz

	// Run action-loop
	while(rclcpp::ok()) {
		// Did the action get cancelled..?
		if(goal_handle->is_canceling()) {
			RCLCPP_INFO(this->get_logger(), "Canceling current waypoint move...");

			// Is there an active sub-action running?
			if(current_client_goal_handle_ != nullptr) {
				// Request cancellation for pilot's move action
				wp_action_client_->async_cancel_goal(current_client_goal_handle_);
			}

			auto result = std::make_shared<WaypointSequence::Result>();
			result->success = false;
			goal_handle->canceled(result);
			return;
		}

		// Did we reach the current waypoint?
		if(reached_waypoint_) {
			work_complete += 1.0;

			// Did we hit the end of the waypoint queue?
			if(quad_route.empty()) {
				// Done, break!
				break;
			}
			else {
				// Request next waypoint
				send_wp_goal(quad_route.front(), waypoint_tolerance);
				quad_route.pop();
			}
		}

		// Update progress
		double progress = 0.0;
		if(dist_total_ > 0) {
			// Add in mid-waypoint progress
			progress = work_complete / total_work +
					  ((1.0 / total_work) * (1.0 - dist_to_waypoint_ / dist_total_));
		}
		else {
			// Just report the waypoint progress
			progress = work_complete / total_work;
		}

		// Report progress
		feedback_msg->progress = progress;
		goal_handle->publish_feedback(feedback_msg);

		if(DEBUG_NAV) {
			RCLCPP_INFO(this->get_logger(), "Progress: %.2f", progress);
		}

		// Sleep to maintain rate
		loop_rate.sleep();
	}

	// Done
	auto result = std::make_shared<WaypointSequence::Result>();
	result->success = true;

	RCLCPP_INFO(this->get_logger(), "WaypointSequence complete.");
	goal_handle->succeed(result);
}



/***************************
 *
 * main function to launch Navigator node
 *
 ***************************/

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);

	auto nav_node = std::make_shared<Navigator>();

    // Spin with MultiThreadedExecutor
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(nav_node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
