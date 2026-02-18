#include "nimbus/planner.h"


/***************************
 *
 * PX4Mission Class
 *
 ***************************/

void PX4Mission::set_mission(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
	// Get mutex
	std::lock_guard<std::mutex> lock(ms_mutex_);
	// Save the waypoint mission
	latest_mission_ = *msg;
	has_mission_ = true;
}

bool PX4Mission::get_waypoint(size_t index, WaypointPosition& out_wp) {
	// Get mutex
	std::lock_guard<std::mutex> lock(ms_mutex_);
	// Input validation
	if(!has_mission_ || !(index < latest_mission_.waypoints.size())) {
		return false;
	}
	// Copy over waypoint data
	const auto& wp = latest_mission_.waypoints[index];
	out_wp = {wp.x_lat, wp.y_long, wp.z_alt};
	return true;
}

// Fills waypoints vector with waypoints that we currently track. Returns false if we do not have a PX4 mission, true o.w.
bool PX4Mission::get_all_waypoints(std::vector<mavros_msgs::msg::Waypoint>* waypoints) {
	// Get mutex
	std::lock_guard<std::mutex> lock(ms_mutex_);
	// Verify we have a current mission
	if(!has_mission_) {
		return false;
	}
	else {
		// Move over all waypoints
		for(auto wp : latest_mission_.waypoints) {
			waypoints->push_back(wp);
		}
		// Return successful
		return true;
	}
}

bool PX4Mission::has_mission() {
	std::lock_guard<std::mutex> lock(ms_mutex_);
	return has_mission_;
}

bool PX4Mission::valid_waypoint(size_t index) {
	// Get mutex
	std::lock_guard<std::mutex> lock(ms_mutex_);
	if(!has_mission_)
		return false;
	// Should be non-zero and less than size of vector
	return index < latest_mission_.waypoints.size();
}


/***************************
 *
 * PX4Geofence Class
 *
 ***************************/

void PX4Geofence::set_fence(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
	// Lock mutex
	std::lock_guard<std::mutex> lock(gf_mutex_);
	// Save geofence data
	latest_fence_ = *msg;
	has_fence_ = true;
}

// Fills vector with geofence waypoints that we currently track. Returns false if we do not have any geofence data, true o.w.
bool PX4Geofence::get_fence_points(std::vector<mavros_msgs::msg::Waypoint>* fence_points) {
	std::lock_guard<std::mutex> lock(gf_mutex_);
	if(!has_fence_) {
		return false;
	}
	else {
		// Move over all waypoints
		for(auto wp : latest_fence_.waypoints) {
			fence_points->push_back(wp);
		}
		return true;
	}
}


bool PX4Geofence::has_fence() {
	std::lock_guard<std::mutex> lock(gf_mutex_);
	return has_fence_;
}


/***************************
 *
 * QuadPosition Class
 *
 ***************************/

void QuadPosition::set_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	// Lock mutex
	std::lock_guard<std::mutex> lock(pos_mutex_);
	// Save position
	latest_position_ = *msg;
	has_position_ = true;
}

bool QuadPosition::get_position(sensor_msgs::msg::NavSatFix& out_pos) {
	// Lock mutex
	std::lock_guard<std::mutex> lock(pos_mutex_);
	// If we have a position...
	if(!has_position_) {
		return false;
	}
	else {
		// Copy over the position
		out_pos = latest_position_;
		return true;
	}
}


/***************************
 *
 * NimbusPlanner Class
 *
 ***************************/

NimbusPlanner::NimbusPlanner() : Node("nimbus_planner") {
	// QoS
	rclcpp::QoS best_effort(10);
	best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);

	/// Subscribers
	// Subscribe to PX4 mission
	mission_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
			"/mavros/mission/waypoints", best_effort,
			std::bind(&NimbusPlanner::mission_callback, this, std::placeholders::_1));
	// Subscribe to Geofence data
	geofence_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
			"/mavros/geofence/fences", best_effort,
			std::bind(&NimbusPlanner::geofence_callback, this, std::placeholders::_1));
	// Subscribe to position
	position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
			"/mavros/global_position/global", best_effort,
			std::bind(&NimbusPlanner::position_callback, this, std::placeholders::_1));

	// Create callback group for concurrent Action Server/Client execution
	cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

	// Move to waypoint action server
	action_server_ = rclcpp_action::create_server<WaypointMove>(
			this,
			"planner/move_to_waypoint",
			std::bind(&NimbusPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&NimbusPlanner::handle_cancel, this, std::placeholders::_1),
			std::bind(&NimbusPlanner::handle_accepted, this, std::placeholders::_1),
			rcl_action_server_get_default_options(),
			cb_group_);

	// Set route action client
	navigator_client_ = rclcpp_action::create_client<WaypointSequence>(
			this, "/nimbus/navigator/set_route", cb_group_);

	// Startup timer for building graph
	startup_timer_ = this->create_wall_timer(
			std::chrono::milliseconds(500),
			std::bind(&NimbusPlanner::startup_check_callback, this));

	RCLCPP_INFO(this->get_logger(), "Nimbus Planner initialized.");
}



////////////////////
//
/// Private
//
////////////////////


// --- Public Interface for Main Loop ---
bool NimbusPlanner::has_graph_data() {
	return px4_mission_.has_mission() && px4_geofence_.has_fence();
}

void NimbusPlanner::build_graph() {
	std::vector<mavros_msgs::msg::Waypoint> mission_wps;
	if(!px4_mission_.get_all_waypoints(&mission_wps)) {
		RCLCPP_WARN(this->get_logger(), "No mission waypoints to build graph.");
		return;
	}

	std::vector<mavros_msgs::msg::Waypoint> fence_wps;
	if(!px4_geofence_.get_fence_points(&fence_wps)) {
		RCLCPP_WARN(this->get_logger(), "No geofence data to build graph.");
		return;
	}

	// Establish coordinate system (Origin = Waypoint 0)
	double origin_lat = mission_wps[0].x_lat;
	double origin_lon = mission_wps[0].y_long;

	// Convert mission nodes to local meters
	std::vector<Point2D> nodes_local;
	for(const auto& wp : mission_wps) {
		nodes_local.push_back(latlon_to_meters(wp.x_lat, wp.y_long, origin_lat, origin_lon));
	}

	// Parse geofences and calculate map bounds
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

	// Store all obstacle polygons
	std::vector<Poly> polygons;

	// State variables
	std::vector<Point2D> current_poly_pts;
	E_MavCMDPolygonType current_type = E_MavCMDPolygonType::EXCLUSIVE;
	int vertices_remaining = 0;

	// For each geofence waypoint
	for(const auto& wp : fence_wps) {
		// Check if we are starting a NEW polygon
		if(vertices_remaining <= 0) {
			// Clear previous points to start fresh
			current_poly_pts.clear();

			// How many vertices are in this polygon?
			vertices_remaining = static_cast<int>(wp.param1);

			// Safety check: avoid infinite loops if param1 is garbage
			if(vertices_remaining <= 0)
				continue;

			// Determine the type for this new polygon
			if(wp.command == E_MavCMDPolygonType::INCLUSIVE) {
				current_type = E_MavCMDPolygonType::INCLUSIVE;
			}
			else if(wp.command == E_MavCMDPolygonType::EXCLUSIVE) {
				current_type = E_MavCMDPolygonType::EXCLUSIVE;
			}
			else {
				// Bad command...
				RCLCPP_WARN(this->get_logger(), "Geofence waypoint not expected type: %d", wp.command);
				return;
			}
		}

		// Process the Point
		Point2D p = latlon_to_meters(wp.x_lat, wp.y_long, origin_lat, origin_lon);

		// Update bounds
		if(p.x < min_x) min_x = p.x;
		if(p.x > max_x) max_x = p.x;
		if(p.y < min_y) min_y = p.y;
		if(p.y > max_y) max_y = p.y;

		// Add point to the current working polygon
		current_poly_pts.push_back(p);

		// Decrement the counter
		vertices_remaining--;

		// Check if we have finished this polygon
		if(vertices_remaining == 0) {
			// Polygon is complete, save it
			polygons.push_back({current_poly_pts, current_type});
		}
	}

	// Create OpenCV map
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

	// Add obstacle polygons
	for(const auto& poly : polygons) {
		if(poly.type == E_MavCMDPolygonType::EXCLUSIVE) {
			std::vector<cv::Point> pts_pix;
			for(auto& p : poly.points) pts_pix.push_back(to_pix(p));

			std::vector<std::vector<cv::Point>> pts_wrapper = {pts_pix};
			cv::fillPoly(obst_map, pts_wrapper, cv::Scalar(255));
		}
	}

	// Dilate obstacles
	int kernel_size = OBSTACLE_DILATION * MAP_RESOLUTION;
	if (kernel_size > 0) {
		 cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
			cv::Size(2*kernel_size+1, 2*kernel_size+1));
		 cv::dilate(obst_map, obst_map, element);
	}

	// Build the graph
	graph_.clear();
	int num_edges = 0;

	for(size_t i = 0; i < nodes_local.size(); ++i) {
		for(size_t j = i + 1; j < nodes_local.size(); ++j) {
			// Distance check
			double d_x = nodes_local[i].x - nodes_local[j].x;
			double d_y = nodes_local[i].y - nodes_local[j].y;
			double dist = std::sqrt(d_x*d_x + d_y*d_y);

			if(dist > MAX_EDGE_LENGTH)
				continue;

			// Collision Check
			cv::Point p1 = to_pix(nodes_local[i]);
			cv::Point p2 = to_pix(nodes_local[j]);

			// Create a line mask for this edge
			cv::LineIterator it(obst_map, p1, p2, 8);
			bool collision = false;
			for(int k = 0; k < it.count; k++, ++it) {
				if(obst_map.at<uchar>(it.pos()) > 0) { // 255 = Obstacle
					collision = true;
					break;
				}
			}

			if(!collision) {
				graph_[i][j] = dist;
				graph_[j][i] = dist;
				num_edges++;
			}
		}
	}

	RCLCPP_INFO(this->get_logger(), "Graph built with %zu nodes and %d edges.", nodes_local.size(), num_edges);
}

// Startup callback to build graph
void NimbusPlanner::startup_check_callback() {
	// Do we have all needed data to build the graph?
	if(this->has_graph_data()) {
		// Yes, build the graph
		RCLCPP_INFO(this->get_logger(), "Building graph...");
		build_graph();

		// Stop timer
		startup_timer_->cancel();

		RCLCPP_INFO(this->get_logger(), "Startup complete. Nimbus Planner ready.");
	}
	else {
		RCLCPP_INFO(this->get_logger(), "Waiting for valid PX4 Mission and Geofence...");
	}
}


void NimbusPlanner::mission_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
	RCLCPP_INFO(this->get_logger(), "Received %zu mission waypoints.", msg->waypoints.size());
	px4_mission_.set_mission(msg);
}

void NimbusPlanner::geofence_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
	RCLCPP_INFO(this->get_logger(), "Received %zu geofence vertices.", msg->waypoints.size());
	px4_geofence_.set_fence(msg);
}

void NimbusPlanner::position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
	quad_position_.set_position(msg);
}


rclcpp_action::GoalResponse NimbusPlanner::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const WaypointMove::Goal> goal) {
	(void)uuid;
	RCLCPP_INFO(this->get_logger(), "Received goal request for WP %d", goal->waypoint);
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NimbusPlanner::handle_cancel(const std::shared_ptr<GoalHandleWaypointMove> goal_handle) {
	(void)goal_handle;
	RCLCPP_INFO(this->get_logger(), "Received cancel request");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void NimbusPlanner::handle_accepted(const std::shared_ptr<GoalHandleWaypointMove> goal_handle) {
	// Execute in new thread
	std::thread{std::bind(&NimbusPlanner::execute, this, std::placeholders::_1), goal_handle}.detach();
}

// Sub-process function for executing waypoint move
void NimbusPlanner::execute(const std::shared_ptr<GoalHandleWaypointMove> goal_handle) {
	auto result = std::make_shared<WaypointMove::Result>();
	auto feedback = std::make_shared<WaypointMove::Feedback>();
	const auto goal = goal_handle->get_goal();

	RCLCPP_INFO(this->get_logger(), "Executing WaypointMove --> %d", goal->waypoint);

	// Check that the navigator is running
	if(!navigator_client_->wait_for_action_server(std::chrono::seconds(5))) {
		RCLCPP_ERROR(this->get_logger(), "Navigator Action Server not available!");
		result->success = false;
		goal_handle->abort(result);
		return;
	}

	// Validate requested waypoint
	if(!px4_mission_.valid_waypoint(goal->waypoint)) {
		RCLCPP_ERROR(this->get_logger(), "Target waypoint %d is invalid.", goal->waypoint);
		result->success = false;
		goal_handle->abort(result);
		return;
	}

	// Get closest waypoint to current position
	int start_idx = get_closest_waypoint();
	if(start_idx == -1) {
		// Failed to find a start point.. we might not have current quad position, or no mission
		RCLCPP_ERROR(this->get_logger(), "Could not determine start index (no GPS/PX4 Mission?)");
		result->success = false;
		goal_handle->abort(result);
		return;
	}

	// Do path planning
	std::vector<int> route;
	if(!calculate_route(start_idx, goal->waypoint, &route)) {
		RCLCPP_WARN(this->get_logger(), "No route found to waypoint %d", goal->waypoint);
		result->success = false;
		goal_handle->abort(result);
		return;
	}

	// Push route into WaypointSequence message
	auto nav_goal = WaypointSequence::Goal();
	for(int wp : route) {
		nav_goal.waypoints.push_back(wp);
	}
	nav_goal.tolerance = goal->tolerance;

	RCLCPP_INFO(this->get_logger(), "Sending route (sized = %zu) to Navigator.", route.size());

	// Send route to nav
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

	// Wait for nav to accept the route
	if(future_goal.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
		 RCLCPP_ERROR(this->get_logger(), "Navigator timed out before accepting goal");
		 result->success = false;
		 goal_handle->abort(result);
		 return;
	}

	// Verify that the nav responded positively...
	auto nav_goal_handle = future_goal.get();
	if(!nav_goal_handle) {
		RCLCPP_ERROR(this->get_logger(), "Navigator rejected route!");
		result->success = false;
		goal_handle->abort(result);
		return;
	}

	// Monitor execution loop
	auto result_future = navigator_client_->async_get_result(nav_goal_handle);

	// Run this feedback thread at 15 Hz
	rclcpp::Rate loop_rate(15.0);

	while(rclcpp::ok()) {
		// Check if someone cancelled move action
		if(goal_handle->is_canceling()) {
			// This action was cancelled...
			RCLCPP_INFO(this->get_logger(), "Canceling Navigator goal...");
			// Tell the navigator to cancel running the route
			navigator_client_->async_cancel_goal(nav_goal_handle);
			result->success = false;
			goal_handle->canceled(result);
			return;
		}

		// Check if the nav finished route action
		if(result_future.wait_for(0s) == std::future_status::ready) {
			auto wrapped_result = result_future.get();
			if(wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
				RCLCPP_INFO(this->get_logger(), "Route execution complete.");
				result->success = true;
				goal_handle->succeed(result);
			}
			else {
				RCLCPP_WARN(this->get_logger(), "Navigator execution failed.");
				result->success = false;
				goal_handle->abort(result);
			}
			return;
		}

		loop_rate.sleep();
	}
}

// --- Helpers ---
int NimbusPlanner::get_closest_waypoint() {
	// Get our current position
	sensor_msgs::msg::NavSatFix pos;
	if(!quad_position_.get_position(pos)) {
        RCLCPP_WARN(this->get_logger(), "get_closest_waypoint() : could not access current position");
		return -1;
	}

	std::vector<mavros_msgs::msg::Waypoint> wps;
	if(!px4_mission_.get_all_waypoints(&wps)) {
        RCLCPP_WARN(this->get_logger(), "get_closest_waypoint() : could not access mission data");
		return -1;
	}

	int best_idx = -1;
	double min_dist = INF;

	// Cycle through each waypoint
	for(size_t i = 0; i < wps.size(); ++i) {
		// Get distance to this waypoint
		double d = get_distance_meters(pos.latitude, pos.longitude, wps[i].x_lat, wps[i].y_long
		);

		// Is this better that current closest?
		if(d < min_dist) {
			// Yes, update
			min_dist = d;
			best_idx = static_cast<int>(i);
		}
	}

	return best_idx;
}

// Converts Lat/Lon to Meters (Local Tangent Plane) relative to an origin
Point2D NimbusPlanner::latlon_to_meters(double lat, double lon, double origin_lat, double origin_lon) {
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
double NimbusPlanner::get_distance_meters(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return EARTH_RADIUS_M * c;
}

bool NimbusPlanner::calculate_route(int start, int end, std::vector<int>* route) {
	// Clear anything in the route vector
	route->clear();

	// Verify that we have a local graph
	if(graph_.empty()) {
		RCLCPP_WARN(this->get_logger(), "calculate_route() : Graph is empty!");
		return false;
	}

	// Validate that the start and end nodes exist in graph and in PX4 Mission
	if(!px4_mission_.valid_waypoint(start)) {
		RCLCPP_WARN(this->get_logger(), "calculate_route() : Bad start waypoint: %d", start);
		return false;
	}
	else if(!px4_mission_.valid_waypoint(end)) {
		RCLCPP_WARN(this->get_logger(), "calculate_route() : Bad target waypoint: %d", end);
		return false;
	}
	else if(graph_.count(start) <= 0) {
		RCLCPP_WARN(this->get_logger(), "calculate_route() : Graph has no data for ID %d", start);
		return false;
	}
	else if(graph_.count(end) <= 0) {
		RCLCPP_WARN(this->get_logger(), "calculate_route() : Graph has no data for ID %d", end);
		return false;
	}
	else {
		// If we made it this far, it should be safe to run A*
		return A_start(start, end, route);
	}
}

double NimbusPlanner::heuristic(int current_id, int goal_id) {
	WaypointPosition current_wp;
	WaypointPosition goal_wp;

	// Fetch waypoints
	if(!px4_mission_.get_waypoint(current_id, current_wp) || !px4_mission_.get_waypoint(goal_id, goal_wp)) {
		return INF;
	}
	else {
		// Return Euclidean distance
		return get_distance_meters(current_wp.latitude, current_wp.longitude, goal_wp.latitude, goal_wp.longitude);
	}
};

bool NimbusPlanner::A_start(int start, int end, std::vector<int>* route) {
	// Create priority queue, track progress
	std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
	std::unordered_map<int, double> g_score;
	std::unordered_map<int, int> came_from;

	// Push start onto queue
	g_score[start] = 0.0;
	open_set.push({start, heuristic(start, end)});

	// While there are still nodes in the priority queue
	while(!open_set.empty()) {
		// Get next node
		int current = open_set.top().id;
		open_set.pop();

		// Did we hit the target?
		if(current == end) {
			// Yes, reconstruct the route
			while (current != start) {
				route->push_back(current);
				current = came_from[current];
			}
			route->push_back(start);
			std::reverse(route->begin(), route->end());
			return true;
		}

		// Explore all neighbors of the current node
		for(auto& edge : graph_[current]) {
			// Calculate new cost
			int neighbor = edge.first;
			double weight = edge.second;
			double tentative_g = g_score[current] + weight;

			// Is this a new node, or a better cost than the previous cost?
			if(g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
				// Add/update cost
				came_from[neighbor] = current;
				g_score[neighbor] = tentative_g;
				double f = tentative_g + heuristic(neighbor, end);
				open_set.push({neighbor, f});
			}
		}
	}

	// If we made it this far, there is no route to the target in the graph
	RCLCPP_WARN(this->get_logger(), "A_start() : Failed to find route %d --> %d", start, end);
	return false;
}



int main(int argc, char **argv) {
	// Create node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NimbusPlanner>();

    // Spin with MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
