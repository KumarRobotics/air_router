#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from router_interfaces.action import WaypointMove, WaypointSequence

DEBUG_PLANNER = True


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

        # Verify that the navigator is up and running
        if not self._navigator_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigator action server not available!")
            goal_handle.abort()
            return WaypointMove.Result(success=False)

        # Extract request data
        target_waypoint = goal_handle.request.waypoint
        tolerance = goal_handle.request.tolerance

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



def main(args=None):
    rclpy.init(args=args)

    planner_node = NimbusPlanner()

    print(f"\tSpinning node...")

    rclpy.spin(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
