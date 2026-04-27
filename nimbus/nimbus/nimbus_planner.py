#!/usr/bin/env python3

# Basic Python stuff
import cv2
import numpy as np
import utm
import heapq
import math
from threading import Lock

# ROS 2 stuff
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix

# MAVROS stuff
from mavros_msgs.msg import WaypointList, Waypoint

# Our stuff
from router_interfaces.action import WaypointMove, WaypointSequence # type: ignore

DEBUG_PLANNER = False

# MAVLink Commands for Geofence
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002


class WaypointPosition:
    """
    WaypointPosition class that is used to move waypoint details between the PX4Mission and 
    anyone else who is curious about the details of a waypoint in the mission.
    """
    def __init__(self, latitude=0.0, longitude=0.0, relative_altitude=0.0):
        self.latitude = latitude
        self.longitude = longitude
        self.relative_altitude = relative_altitude

    def __repr__(self):
        return f"Lat: {self.latitude}, Lon: {self.longitude}, RelAlt: {self.relative_altitude}"


class PX4Mission:
    """
    Manages storage and retrieval of the PX4 mission, which is stored as a WaypointList. This 
    class is used instead of storing the WaypointList directly for thread safety.
    """
    def __init__(self):
        self._latest_mission = None
        self._has_mission = False
        self._mission_mutex = Lock()

    def set_mission(self, msg: WaypointList):
        """
        Store latest mission message safely.
        """
        with self._mission_mutex:
            self._latest_mission = msg
            self._has_mission = True

    def get_mission(self):
        """
        Returns the full WaypointList if available, else None.
        """
        with self._mission_mutex:
            if not self._has_mission:
                return None
            return self._latest_mission

    def get_num_waypoints(self):
        """
        Returns the number of waypoints in the current mission.
        """
        with self._mission_mutex:
            if not self._has_mission:
                return 0
            else:
                return len(self._latest_mission.waypoints)

    def get_waypoint(self, wp_id: int):
        """
        Returns a WaypointPosition object for the specific ID if valid.
        Returns None if invalid or no mission exists.
        """
        with self._mission_mutex:
            if not self._has_mission:
                return None

            # Check bounds (valid_waypoint logic)
            if 0 <= wp_id < len(self._latest_mission.waypoints):
                wp = self._latest_mission.waypoints[wp_id]
                # Map MAVROS fields: x_lat, y_long, z_alt
                return WaypointPosition(
                    latitude=wp.x_lat,
                    longitude=wp.y_long,
                    relative_altitude=wp.z_alt
                )
            else:
                return None

    def get_all_waypoints(self):
        """Returns a list of all waypoints for planning."""
        with self._mission_mutex:
            if not self._has_mission:
                return []
            return self._latest_mission.waypoints

    def valid_waypoint(self, wp_id: int) -> bool:
        """
        Checks if wp_id exists in the mission.
        """
        with self._mission_mutex:
            if not self._has_mission:
                return False
            return 0 <= wp_id < len(self._latest_mission.waypoints)

    def has_mission(self) -> bool:
        """
        Checks if wp_id exists in the mission.
        """
        with self._mission_mutex:
            return self._has_mission


class PX4Geofence:
    """
    Manages storage and retrieval of the geofences (no-fly zones), which are stored as 
    a WaypointList. This class is used instead of storing the WaypointList directly for 
    thread safety.
    """
    def __init__(self):
        self._latest_fence = None
        self._has_fence = False
        self._fence_mutex = Lock()

    def set_fence(self, msg: WaypointList):
        with self._fence_mutex:
            self._latest_fence = msg
            self._has_fence = True

    def get_fence_points(self):
        """
        Returns the raw list of fence waypoints.
        """
        with self._fence_mutex:
            if not self._has_fence:
                return []
            return self._latest_fence.waypoints

    def has_fence(self):
        """
        Returns true if we have a fence
        """
        with self._fence_mutex:
            return self._has_fence


class QuadPosition:
    """
    Thread-safe wrapper for tracking most recently published position message
    """
    def __init__(self):
        self._latest_position = None
        self._pos_mutex = Lock()

    def set_position(self, msg: NavSatFix):
        """
        Store position message
        """
        with self._pos_mutex:
            self._latest_position = msg

    def get_position(self):
        """
        Returns the latest NavSatFix message if available, else None.
        """
        with self._pos_mutex:
            return self._latest_position


class NimbusPlanner(Node):

    def __init__(self):
        super().__init__('planner')

        # Create a ReentrantCallbackGroup to allow the server and client 
        # to process callbacks concurrently (essential for cancellation to work)
        self._cb_group = ReentrantCallbackGroup()

        # move_to_waypoint Action Server
        self._action_server = ActionServer(
            self,
            WaypointMove,
            'planner/move_to_waypoint',
            execute_callback=self.route_execute_callback,
            goal_callback=self.route_goal_callback,
            cancel_callback=self.route_cancel_callback
        )

        # Nav's set_route Action Client
        self._navigator_client = ActionClient(
            self,
            WaypointSequence,
            '/nimbus/navigator/set_route'
        )

        # Track the active handle for the client so we can cancel it if needed
        self._current_navigator_goal_handle = None
        # Track when we finished a route
        self.finished_route = False
        # Track how far we are along the route
        self.route_progress = 0.0

        # Internal helper to manage PX4 mission/geofence
        self.px4_mission = PX4Mission()
        self.px4_geofence = PX4Geofence()

        # QoS for talking to PX4 (KeepLast(10), Best Effort)
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribe to mission waypoints from PX4
        self.mission_wp_sub = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.mission_wp_callback,
            qos_profile
        )

        # Subscribe to geofence data from PX4
        self.geofence_sub = self.create_subscription(
            WaypointList,
            '/mavros/geofence/fences',
            self.geofence_callback,
            qos_profile
        )

        # Initialize position helper
        self.quad_position = QuadPosition()

        # Subscribe to global position
        self.position_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.position_callback,
            qos_profile
        )
        self.dbg_pos_count = 0

        # Planning Constants
        self.MAX_EDGE_LENGTH = 500  # Meters
        self.MAP_RESOLUTION = 10.0  # Pixels per meter
        self.MAP_BUFFER = 50        # Buffer around mission area in meters
        self.OBSTACLE_DILATION = 1  # Obstacle dilation, in meters
        self.graph = None

        # Wait for the PX4 mission and geofence
        while rclpy.ok() and not self.px4_mission.has_mission():
            self.get_logger().info(
                "Waiting for valid PX4 Mission...", 
                throttle_duration_sec=1.0
            )
            rclpy.spin_once(self, timeout_sec=1.0)
        while rclpy.ok() and not self.px4_geofence.has_fence():
            self.get_logger().info(
                "Waiting for valid geofence...", 
                throttle_duration_sec=1.0
            )
            rclpy.spin_once(self, timeout_sec=1.0)

        self.build_graph()

        self.get_logger().info("Nimbus Planner is ready.")


    def route_goal_callback(self, goal_request):
        """Accept the incoming goals."""
        self.get_logger().info(f"Planner received request for Waypoint: {goal_request.waypoint}")
        
        # Just say yes...
        return rclpy.action.GoalResponse.ACCEPT


    def route_cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info("Planner received cancel request.")
        return CancelResponse.ACCEPT


    async def route_execute_callback(self, goal_handle):
        self.get_logger().info("Executing WaypointMove action...")

        # Extract request data
        target_waypoint = goal_handle.request.waypoint
        tolerance = goal_handle.request.tolerance

        # Verify that the navigator is up and running
        if not self._navigator_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigator action server not available!")
            goal_handle.abort()
            return WaypointMove.Result(success=False)
        # Verify that this is a valid waypoint
        elif not self.px4_mission.valid_waypoint(target_waypoint):
            self.get_logger().error(f"Requested bad waypoint ({target_waypoint})!")
            goal_handle.abort()
            return WaypointMove.Result(success=False)

        # --- Calculate Route ---
        start_node_index = self.get_closest_waypoint()

        if DEBUG_PLANNER:
            print(f"Planning path from {start_node_index} to {target_waypoint}")
        
        route_sequence = self.calculate_route(start_node_index, target_waypoint)
        
        # Did we find a valid route?
        if not route_sequence:
            # No, give up!
            self.get_logger().warn(f"Could not find a route to waypoint {target_waypoint}")
            goal_handle.abort()
            result = WaypointMove.Result()
            result.success = False
            return result
        
        self.get_logger().info(f"Generated route: {route_sequence}")

        # Send first waypoint
        self.send_wp_route(route_sequence, tolerance)

        feedback_msg = WaypointMove.Feedback()

        # Run action-loop
        while rclpy.ok():
            # Did the action get cancelled..?
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling current route...")
                # Is there an active sub-action running?
                if self._current_navigator_goal_handle is not None:
                    # Request cancellation of the sub-action
                    future = self._current_navigator_goal_handle.cancel_goal_async()
                # Cancel this goal
                goal_handle.canceled()
                return WaypointMove.Result(success=False)

            # Report progress
            feedback_msg.distance_to_go = self.route_progress
            goal_handle.publish_feedback(feedback_msg)

            # Did we reach the current waypoint?
            if self.finished_route:
                break

            # Spin ROS
            rclpy.spin_once(self)

        # Done
        result = WaypointMove.Result()
        result.success = True

        self.get_logger().info("Route complete.")
        goal_handle.succeed()
        return result


    def navigator_feedback_callback(self, feedback_msg):
        """
        Receive progress from Navigator (WaypointSequence)
        and (optionally) relay it to the User (WaypointMove).
        """
        progress_pct = feedback_msg.feedback.progress
        self.get_logger().info(f"Navigator Progress: {progress_pct*100:.1f}%")


    # --- Send the route to the navigator --------------------------------
    def send_wp_route(self, route_sequence, tolerance: float):
        # Wait for the action server to come online
        self._navigator_client.wait_for_server()

        # Create navigator action request
        nav_goal = WaypointSequence.Goal()
        nav_goal.waypoints = route_sequence
        nav_goal.tolerance = tolerance

        # Goal tracking
        self.finished_route = False
        self.route_progress = 0.0

        # Logging
        self.get_logger().info(f'Sending navigator route (|{len(route_sequence)}|), tolerance = {tolerance}')

        # Send goal asynchronously
        self._send_goal_future = self._navigator_client.send_goal_async(
            nav_goal,
            feedback_callback=self.wp_feedback_callback
        )

        # Attach callback for request result
        self._send_goal_future.add_done_callback(self.wp_response_callback)


    # --- Route request feedback -----------------------------------------
    def wp_feedback_callback(self, feedback_msg):
        # Record how far we have come
        feedback = feedback_msg.feedback
        self.route_progress = feedback.progress
        
        # Logging
        if DEBUG_PLANNER:
            self.get_logger().info(f'Progress: {(feedback.progress*100):.1f}%')


    # --- Route request response -----------------------------------------
    def wp_response_callback(self, future):
        goal_handle = future.result()

        # Did the nav reject the goal?
        if not goal_handle.accepted:
            self.get_logger().warn('Route rejected')
            self.finished_route = True
            return

        self.get_logger().info('Route accepted')

        # Store the action handle (used to cancel action, if needed)
        self._current_navigator_goal_handle = goal_handle

        # Attach result callback
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.wp_result_callback)


    # --- Route request final result -------------------------------------
    def wp_result_callback(self, future):
        result = future.result().result
        # Regardless of the result, mark that the action ended
        self.finished_route = True

        self.get_logger().info(f'Waypoint action result: {result.success}')


    def mission_wp_callback(self, msg: WaypointList):
        """
        Callback for /mavros/mission/waypoints
        """
        self.get_logger().info(f"Received {len(msg.waypoints)} mission waypoints.")

        # Store the mission
        self.px4_mission.set_mission(msg)

        # Logging
        if DEBUG_PLANNER:
            for i, wp in enumerate(msg.waypoints):
                self.get_logger().info(f"WP {i}: lat={wp.x_lat} lon={wp.y_long} alt={wp.z_alt}")


    def geofence_callback(self, msg: WaypointList):
        """
        Callback for /mavros/geofence/fences
        """
        self.get_logger().info(f"Received {len(msg.waypoints)} geofence vertices.")
        self.px4_geofence.set_fence(msg)


    def position_callback(self, msg: NavSatFix):
        """
        Callback for /mavros/global_position/global
        """
        if DEBUG_PLANNER:
            self.dbg_pos_count += 1
            if self.dbg_pos_count % 20 == 0:
                self.get_logger().info(
                    f"Received NavSatFix: lat={msg.latitude:.6f} lon={msg.longitude:.6f} alt={msg.altitude:.2f}"
                )

        # Record current position
        self.quad_position.set_position(msg)


    # --- Graph building logic --------------------------------------------
    def build_graph(self):
        """
        Builds waypoint graph with distance as edge weights, uses OpenCV for obstacle mapping.
        """
        self.get_logger().info(f"Building waypoint graph")
        
        # Get mission data
        mission_wps = self.px4_mission.get_all_waypoints()
        fence_wps = self.px4_geofence.get_fence_points()

        # Verify that we actually have a waypoints/fence
        if mission_wps == None or fence_wps == None:
            self.get_logger().warn("No mission waypoints or fence!")
            return

        # Establish coordinate system using UTM
        # Use WP 0 as the origin for local grid
        origin_lat = mission_wps[0].x_lat
        origin_lon = mission_wps[0].y_long
        origin_utm = utm.from_latlon(origin_lat, origin_lon)
        
        # Helper to convert lat/lon to local meters
        def to_local(lat, lon):
            u = utm.from_latlon(lat, lon)
            return (u[0] - origin_utm[0], u[1] - origin_utm[1])

        # Process mission nodes
        nodes = {} # {index: (x, y)}
        for i, wp in enumerate(mission_wps):
            x, y = to_local(wp.x_lat, wp.y_long)
            nodes[i] = (x, y)

        # Determine map bounds for image generation
        all_x = [p[0] for p in nodes.values()]
        all_y = [p[1] for p in nodes.values()]
        
        # Add fence points to bounds check
        fence_polygons = self._parse_geofence(fence_wps, to_local)
        for poly in fence_polygons:
            for pt in poly['points']:
                all_x.append(pt[0])
                all_y.append(pt[1])

        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        width_m = max_x - min_x + (2 * self.MAP_BUFFER)
        height_m = max_y - min_y + (2 * self.MAP_BUFFER)
        
        img_w = int(width_m * self.MAP_RESOLUTION)
        img_h = int(height_m * self.MAP_RESOLUTION)
        
        # Offset to map pixels
        offset_x = -min_x + self.MAP_BUFFER
        offset_y = -min_y + self.MAP_BUFFER

        def to_pix(x_m, y_m):
            px = int((x_m + offset_x) * self.MAP_RESOLUTION)
            # Flip Y for image coordinates
            py = int(img_h - ((y_m + offset_y) * self.MAP_RESOLUTION)) 
            return px, py

        # Draw obstacles (Mask)
        # Initialize white image (safe), draw black obstacles
        # Or: 0 = safe, 255 = obstacle
        obst_map = np.zeros((img_h, img_w), dtype=np.uint8)

        for poly in fence_polygons:
            pts_pix = []
            for pt in poly['points']:
                pts_pix.append(to_pix(pt[0], pt[1]))
            
            pts_np = np.array([pts_pix], dtype=np.int32)
            
            if poly['type'] == 'exclusion':
                cv2.fillPoly(obst_map, pts_np, 255) # 255 is obstacle
            elif poly['type'] == 'inclusion':
                # We assume that inclusive geofences are convex hulls and that all 
                # waypoints are within these convex hulls
                pass

        # Dilate obstacles for safety margin (e.g., 1 meter)
        kernel_size = int(self.OBSTACLE_DILATION * self.MAP_RESOLUTION) 
        if kernel_size > 0:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            obst_map = cv2.dilate(obst_map, kernel, iterations=1)

        # Build the graph
        self.graph = {i: {} for i in nodes}
        
        for i in nodes:
            for j in nodes:
                if i == j: continue
                
                # Check distance
                p1 = np.array(nodes[i])
                p2 = np.array(nodes[j])
                dist = np.linalg.norm(p1 - p2)
                
                if dist > self.MAX_EDGE_LENGTH:
                    continue

                # Check collision using Line Iterator or Drawing
                pix1 = to_pix(nodes[i][0], nodes[i][1])
                pix2 = to_pix(nodes[j][0], nodes[j][1])
                
                # Draw line on a temp mask to check intersection
                # Optimization: use LineIterator or bitwise check
                line_mask = np.zeros_like(obst_map)
                cv2.line(line_mask, pix1, pix2, 255, 1) # 1px width check
                
                intersection = cv2.bitwise_and(line_mask, obst_map)
                if cv2.countNonZero(intersection) == 0:
                    # Safe path
                    self.graph[i][j] = dist


    def _parse_geofence(self, fence_wps, coord_transform_func):
        """
        Groups fence waypoints into polygons based on MAV_CMD.
        Returns list of dicts: {'type': 'exclusion'/'inclusion', 'points': [(x,y), ...]}
        """
        polygons = []
        current_poly = []
        current_type = 'exclusion' # Default
        
        for wp in fence_wps:
            # Check for new polygon start or continuation
            # Logic depends on how MAVROS/QGC serializes lists.
            # Often it's just a sequence of points.
            # We look for the Command ID.
            
            p_local = coord_transform_func(wp.x_lat, wp.y_long)
            
            if wp.command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                # If we were building an inclusion poly, save it
                if current_type == 'inclusion' and current_poly:
                     polygons.append({'type': 'inclusion', 'points': current_poly})
                     current_poly = []
                current_type = 'exclusion'
                current_poly.append(p_local)
                
            elif wp.command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                 if current_type == 'exclusion' and current_poly:
                     polygons.append({'type': 'exclusion', 'points': current_poly})
                     current_poly = []
                 current_type = 'inclusion'
                 current_poly.append(p_local)
            else:
                # Generic fallback: assume it belongs to the current polygon
                # or treat as exclusion if unknown
                current_poly.append(p_local)

        # Append last one
        if current_poly:
            polygons.append({'type': current_type, 'points': current_poly})

        # Note: This simple parser assumes consecutive vertices define a polygon.
        # Robust implementations might check param1 (vertex count).
        return polygons


    # --- Find closest waypoint --------------------------------------------
    def get_closest_waypoint(self):
        """
        Determine which waypoint the quad is currently closest to. Returns None if we do 
        not have a PX4 mission or position.
        """
        # Get latest mission and current position
        current_pos = self.quad_position.get_position()
        mission_wps = self.px4_mission.get_all_waypoints()

        # Verify that we useful data
        if current_pos == None:
            self.get_logger().warn("Failed to get closest waypoint -> no position data")
            return None
        if mission_wps == None:
            self.get_logger().warn("Failed to get closest waypoint -> no mission data")
            return None

        closest_index = -1
        min_dist = float('inf')
        R = 6371000.0  # Earth radius in meters

        # Calculate current position in radians
        lat1 = math.radians(current_pos.latitude)
        lon1 = math.radians(current_pos.longitude)

        # Iterate over all waypoints
        for i, wp in enumerate(mission_wps):
            # MAVROS Waypoints use x_lat, y_long
            lat2 = math.radians(wp.x_lat)
            lon2 = math.radians(wp.y_long)

            # Haversine Formula
            dlat = lat2 - lat1
            dlon = lon2 - lon1

            a = math.sin(dlat / 2)**2 + \
                math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
            
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            dist = R * c

            # Update minimum
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        return closest_index


    # --- Path Planning Logic --------------------------------------------
    def calculate_route(self, start_idx, end_idx):
        """
        Uses OpenCV for obstacle mapping and Dijkstra for routing.
        """
        # Verify that we have a graph and were given valid points
        if self.graph == None:
            self.get_logger().warn(f"Path planner has no graph!")
            return None
        elif not self.px4_mission.valid_waypoint(start_idx):
            self.get_logger().warn(f"Bad start point ({start_idx})")
            return None
        elif not self.px4_mission.valid_waypoint(end_idx):
            self.get_logger().warn(f"Bad end point ({end_idx})")
            return None

        # Run Dijkstra's algorithm
        return self._astar(start_idx, end_idx)


    def _dijkstra(self, start, end):
        queue = [(0, start, [])]
        seen = set()
        min_dist = {start: 0}

        while queue:
            (cost, v1, path) = heapq.heappop(queue)
            
            if v1 in seen:
                continue
            seen.add(v1)

            path = path + [v1]
            if v1 == end:
                return path

            for v2, weight in self.graph.get(v1, {}).items():
                if v2 in seen:
                    continue
                prev = min_dist.get(v2, None)
                next_cost = cost + weight
                if prev is None or next_cost < prev:
                    min_dist[v2] = next_cost
                    heapq.heappush(queue, (next_cost, v2, path))

        return None
    
    def _astar(self, start, end):
        """
        A* Search Algorithm using PX4Mission for live coordinate data.
        """
        
        # Fetch the target waypoint 
        target_wp = self.px4_mission.get_waypoint(end)
        if not target_wp:
            self.get_logger().error(f"Target waypoint {end} not found in mission.")
            return None
        
        # Convert this to lat/long
        target_utm = utm.from_latlon(target_wp.latitude, target_wp.longitude)

        # Heuristic Function: Euclidean distance in meters from n_idx to end
        def h(n_idx):
            # Get waypoint
            wp = self.px4_mission.get_waypoint(n_idx)
            if not wp:
                return float('inf')
            
            # Convert to UTM (m)
            wp_utm = utm.from_latlon(wp.latitude, wp.longitude)
            
            # Euclidean distance
            return math.hypot(target_utm[0] - wp_utm[0], target_utm[1] - wp_utm[1])

        # Priority Queue: (f_score, g_score, current_node, path)
        start_h = h(start)
        queue = [(start_h, 0, start, [])]
        
        seen = set()
        min_g = {start: 0}

        # While there are still nodes in the queue
        while queue:
            (f, g, u, path) = heapq.heappop(queue)

            if u in seen:
                continue
            seen.add(u)

            path = path + [u]

            if u == end:
                return path

            # Explore neighbors
            for v, weight in self.graph.get(u, {}).items():
                if v in seen:
                    continue
                
                new_g = g + weight
                
                # If we found a shorter path to v
                if new_g < min_g.get(v, float('inf')):
                    min_g[v] = new_g
                    new_f = new_g + h(v) # Calculate fresh heuristic
                    heapq.heappush(queue, (new_f, new_g, v, path))

        # If we made it this far.. we failed...
        self.get_logger().warn(f"Failed to find path to goal ({end})")
        return None



def main(args=None):
    rclpy.init(args=args)

    planner_node = NimbusPlanner()

    print(f"\tSpinning node...")

    rclpy.spin(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
