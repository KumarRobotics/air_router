#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.msg import WaypointList
from threading import Lock

from router_interfaces.action import WaypointMove, WaypointSequence


DEBUG_PLANNER = True


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

    def valid_waypoint(self, wp_id: int) -> bool:
        """
        Checks if wp_id exists in the mission.
        """
        with self._mission_mutex:
            if not self._has_mission:
                return False
            return 0 <= wp_id < len(self._latest_mission.waypoints)


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

        # Internal helper to manage PX4 mission data
        self.px4_mission = PX4Mission()

        # QoS for talking to PX4 (KeepLast(10), Best Effort)
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribe to mission data from PX4
        self.mission_wp_sub = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.mission_wp_callback,
            qos_profile
        )

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

        # Calculate route to desired waypoint
        route_sequence = self.calculate_route(target_waypoint)
        
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
        # Note: WaypointSequence returns 'progress' (0.0 to 1.0)
        # But WaypointMove expects 'distance_to_go' (meters).
        # Since we don't know the physical distance here easily, 
        # we might log it or leave it 0.0.
        
        progress_pct = feedback_msg.feedback.progress
        self.get_logger().info(f"Navigator Progress: {progress_pct*100:.1f}%")
        
        # If you wanted to send feedback back up:
        # feedback = WaypointMove.Feedback()
        # feedback.distance_to_go = 0.0 # Unknown
        # self.get_logger().info("Relaying feedback...") 
        # (This requires passing the goal_handle to this callback, 
        #  which is complex without a lambda or partial).


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


    def calculate_route(self, target_waypoint):
        """
        YOUR LOGIC HERE.
        Returns a list of integers (waypoints).
        """
        # Placeholder: just return a simple list ending in the target
        self.get_logger().info(f"Calculating route to {target_waypoint}...")
        
        # Example logic: [1, 2, ..., target]
        # Only for demonstration
        return [19, target_waypoint]


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




def main(args=None):
    rclpy.init(args=args)

    planner_node = NimbusPlanner()

    print(f"\tSpinning node...")

    rclpy.spin(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
