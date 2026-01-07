#include "nimbus/pilot.h"


/***************************
 *
 * QuadPosition Class
 *
 ***************************/

// Store latest position message
void QuadPosition::set_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(pos_mutex_);
	latest_position_ = *msg;

	// Is this our first position..?
	if(!has_position_) {
		// Record this position
		initial_position_ = *msg;
		has_position_ = true;
	}
}

// Getter:
//  - copies stored latitude/longitude/altitude into *out_msg
//  - returns true if at least one message has been stored
bool QuadPosition::get_position(sensor_msgs::msg::NavSatFix* out_msg) {
	// Verify there exists a NavSatFix object
	if(out_msg == nullptr) {
		return false;
	}

	// Lock the mutex
	std::lock_guard<std::mutex> lock(pos_mutex_);

	// Have we received at least on position message?
	if(!has_position_) {
		// No...
		return false;
	}
	else {
		// Yes, return the most recent
		*out_msg = latest_position_;
		return true;
	}
}

// Computes the distance from the quads current position to the given lat/long/alt
double QuadPosition::distance_to(Waypoint_Position* wp_pos) {
	// Assume infinite distance
	double dist_to_go = INF;

	// Get the current position
	sensor_msgs::msg::NavSatFix current_position;
	if(has_position_ && get_position(&current_position)) {
		// Convert latitude and longitude to radians
		double req_lat_rad = degToRad(wp_pos->latitude);
		double req_lon_rad = degToRad(wp_pos->longitude);
		double crnt_lat_rad = degToRad(current_position.latitude);
		double crnt_lon_rad = degToRad(current_position.longitude);

		// Distance to earth's center
		double req_radius = EARTH_RADIUS_M + wp_pos->relative_altitude;
		// Use quad relative position
		double crnt_radius = EARTH_RADIUS_M + (current_position.altitude - initial_position_.altitude);

		// Convert to Cartesian points
		double req_x = req_radius * cos(req_lat_rad) * cos(req_lon_rad);
		double req_y = req_radius * cos(req_lat_rad) * sin(req_lon_rad);
		double req_z = req_radius * sin(req_lat_rad);
		double crnt_x = crnt_radius * cos(crnt_lat_rad) * cos(crnt_lon_rad);
		double crnt_y = crnt_radius * cos(crnt_lat_rad) * sin(crnt_lon_rad);
		double crnt_z = crnt_radius * sin(crnt_lat_rad);

		// Calculate the distance between these two points
	    double dx = req_x - crnt_x;
	    double dy = req_y - crnt_y;
	    double dz = req_z - crnt_z;
	    dist_to_go = sqrt(dx * dx + dy * dy + dz * dz);
	}

	// Return what we found
	return dist_to_go;
}

double QuadPosition::degToRad(double degree) {
	return degree * (M_PI / 180.0);
}


/***************************
 *
 * PX4Mission Class
 *
 ***************************/

// Store latest mission message
void PX4Mission::set_mission(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(mission_mutex_);
	latest_mission_ = *msg;
	has_mission_ = true;
}

// Getter:
//  - copies the current mission into *out_msg
//  - returns true if at least one mission message has been stored
bool PX4Mission::get_mission(mavros_msgs::msg::WaypointList* out_msg) {
	// Verify the input object exists
	if(out_msg == nullptr) {
		return false;
	}

	// Lock the mutex
	std::lock_guard<std::mutex> lock(mission_mutex_);

	// Have we received at least on position message?
	if(!has_mission_) {
		// No...
		return false;
	}
	else {
		// Yes, return the most recent
		*out_msg = latest_mission_;
		return true;
	}
}

// Copies the details of waypoint wp_id into pos, returns true if successful
bool PX4Mission::get_waypoint(int wp_id, Waypoint_Position* pos) {
	// Verify the input object exists
	if(pos == nullptr) {
		return false;
	}

	// Lock the mutex
	std::lock_guard<std::mutex> lock(mission_mutex_);

	// Have we received at least on position message?
	if(!has_mission_) {
		// No...
		return false;
	}
	else {
		// Yes, is this waypoint in range?
		if(wp_id >= 0 && wp_id < (int)(latest_mission_.waypoints.size() & INT_MAX)) {
			// Yes, copy over lat/long/alt
			(*pos).latitude = latest_mission_.waypoints[wp_id].x_lat;
			(*pos).longitude= latest_mission_.waypoints[wp_id].y_long;
			(*pos).relative_altitude = latest_mission_.waypoints[wp_id].z_alt;
		}
		else {
			return false;
		}
	}

	return true;
}



/***************************
 *
 * Pilot Class
 *
 ***************************/

Pilot::Pilot() : Node("pilot") {
	// Subscribe to global position
	position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
			"/mavros/global_position/global",					// Topic name
			rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),	// QoS history depth
			std::bind(&Pilot::position_callback, this, std::placeholders::_1)
	);
	// Subscribe to mission data
	mission_wp_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
			"/mavros/mission/waypoints",					// Topic name
			rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),	// QoS history depth
			std::bind(&Pilot::mission_wp_callback, this, std::placeholders::_1)
	);
	set_wp_client_ = this->create_client<mavros_msgs::srv::WaypointSetCurrent>("/mavros/mission/set_current");
	pull_wp_client_ = this->create_client<mavros_msgs::srv::WaypointPull>("/mavros/mission/pull");

	// Action server
	set_wp_action_server_ = rclcpp_action::create_server<WaypointMove>(
			this,
			"pilot/set_waypoint",
			std::bind(&Pilot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Pilot::handle_cancel, this, std::placeholders::_1),
			std::bind(&Pilot::handle_accepted, this, std::placeholders::_1));

	// Wall timers

	RCLCPP_INFO(this->get_logger(), "Pilot node initialized.");
}

// Set the target waypoint on the quad through MAVROS
void Pilot::set_waypoint(uint16_t waypoint_index) {
	// Wait for service to become available
	if (!set_wp_client_->wait_for_service(std::chrono::seconds(2))) {
		RCLCPP_ERROR(this->get_logger(), "Service /mavros/mission/set_current not available.");
		return;
	}

	auto request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
	request->wp_seq = waypoint_index;

	RCLCPP_INFO(this->get_logger(), "Setting target waypoint (%d) through MAVROS", waypoint_index);

	// Asynchronous call with callback lambda
	auto future_result = set_wp_client_->async_send_request(
		request,
		std::bind(
			&Pilot::set_waypoint_callback,
			this,
			std::placeholders::_1
		)
	);
}

// Global position callback
void Pilot::position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	if(DEBUG_PILOT) {
		RCLCPP_INFO(this->get_logger(),
				"Received NavSatFix: lat=%.6f  lon=%.6f  alt=%.2f",
				msg->latitude, msg->longitude, msg->altitude);
	}

	// Record current position
	quadPosition.set_position(msg);
}

// Mission waypoints callback
void Pilot::mission_wp_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
	if(DEBUG_PILOT) {
		RCLCPP_INFO(this->get_logger(), "Received %zu mission waypoints.", msg->waypoints.size());
		for(size_t i = 0; i < msg->waypoints.size(); i++) {
			const auto &wp = msg->waypoints[i];
			RCLCPP_INFO(this->get_logger(), "WP %zu: lat=%f lon=%f alt=%f cmd=%d",
				i, wp.x_lat, wp.y_long, wp.z_alt, wp.command);
		}
	}

	// Record the mission file
	px4Mission.set_mission(msg);
}

void Pilot::set_waypoint_callback(rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedFuture result) {
	const auto response = result.get();
	if(response->success) {
		RCLCPP_INFO(this->get_logger(), "MAVROS accepted next waypoint.");
	}
	else {
		RCLCPP_WARN(this->get_logger(), "MAVROS rejected next waypoint.");
	}
}

// Request mission waypoints from MAVROS/PX4
/// WARNING: This function is now deprecated. Only nodes that set the FC mission should request a waypoint pull.
void Pilot::pull_waypoints_timer_callback() {
	// This is deprecated --> make a fuss!
	RCLCPP_WARN(this->get_logger(), "Pilot::pull_waypoints_timer_callback is deprecated!");

	if(!pull_wp_client_->wait_for_service(std::chrono::seconds(1))) {
		RCLCPP_WARN(this->get_logger(), "Waiting for /mavros/mission/pull service...");
		return;
	}

	// Send request
	auto request = std::make_shared<mavros_msgs::srv::WaypointPull::Request>();
	pull_wp_client_->async_send_request(request,
			[this](rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedFuture future)
			{
				if(future.get()->success) {
					if(DEBUG_PILOT)
						RCLCPP_WARN(this->get_logger(), "Requested mission data");
				}
				else {
					RCLCPP_WARN(this->get_logger(), "Mission pull failed.");
				}
			}
	);
}

// WP Action accept goal callback -- Blindly accepts all goals
rclcpp_action::GoalResponse Pilot::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const WaypointMove::Goal> goal) {
	// Suppress warnings...
	(void)uuid;
	RCLCPP_INFO(this->get_logger(), "Received waypoint goal request: %d", goal->waypoint);

	// Just accept goal
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// WP Action cancel callback -- does nothing but should probably stop the quad
rclcpp_action::CancelResponse Pilot::handle_cancel(const std::shared_ptr<GoalHandleWaypointMove> goal_handle) {
	// Suppress warnings...
	(void)goal_handle;
	RCLCPP_WARN(this->get_logger(), "Canceling waypoint action");

	// TODO: Should probably stop the quad or something
	return rclcpp_action::CancelResponse::ACCEPT;
}

// WP Action execution callback -- starts a new thread and returns
void Pilot::handle_accepted(const std::shared_ptr<GoalHandleWaypointMove> goal_handle) {
	// Run on a separate thread so the executor isn't blocked
	std::thread{std::bind(&Pilot::execute, this, goal_handle)}.detach();
}

// Worker function to monitor waypoint move. Returns when the quad reaches the waypoint or action is canceled.
void Pilot::execute(const std::shared_ptr<GoalHandleWaypointMove> goal_handle) {
	// Get goal and feedback handles
	const std::shared_ptr<const router_interfaces::action::WaypointMove::Goal> goal = goal_handle->get_goal();
	auto feedback = std::make_shared<WaypointMove::Feedback>();
	auto result = std::make_shared<WaypointMove::Result>();

	RCLCPP_INFO(this->get_logger(), "Executing waypoint action, move to %d -- tolerance: %.2f", goal->waypoint, goal->tolerance);

	// Actually send the waypoint command
	set_waypoint(goal->waypoint);

	// Run this feedback thread at 4 Hz
	rclcpp::Rate loop_rate(4.0);

	// While ROS is still running...
	while(rclcpp::ok()) {
		// Did the action get canceled..?
		if(goal_handle->is_canceling()) {
			result->success = false;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Waypoint action canceled.");
			return;
		}

		// Get the target position
		Waypoint_Position target_pos;
		if(px4Mission.get_waypoint(goal->waypoint, &target_pos)) {
			// Compute current distance to the target
			double distance_to_go = quadPosition.distance_to(&target_pos);

			// Send feedback
			feedback->distance_to_go = distance_to_go;
			goal_handle->publish_feedback(feedback);

			if(DEBUG_PILOT) {
				RCLCPP_INFO(this->get_logger(), "Distance to go: %.2f", distance_to_go);
			}

			// Did we reach the target?
			if(distance_to_go <= goal->tolerance) {
				// Action complete.. break out of the while-loop
				break;
			}
		}
		else {
			// Slowly complain about not finding the waypoint
			static int counter = 0;
			if(counter%10 == 0) {
				RCLCPP_WARN(this->get_logger(), "Not able to find waypoint %d!", goal->waypoint);
			}
			counter++;
		}

		loop_rate.sleep();
	}

	// If ROS is still alive...
	if(rclcpp::ok()) {
		// Report that the action was successful!
		result->success = true;
		goal_handle->succeed(result);
		RCLCPP_INFO(this->get_logger(), "Reached waypoint successfully!");
	}
}


/***************************
 *
 * main
 *
 ***************************/

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Pilot>());
	rclcpp::shutdown();
	return 0;
}
