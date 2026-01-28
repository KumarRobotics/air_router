#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import CancelResponse

from router_interfaces.action import WaypointMove, WaypointSequence

"""Navigator:
This node handles the current waypoint queue. It removes waypoints from the 
queue and feeds them to the pilot. As the pilot reaches each waypoint, the 
navigator pulls the next waypoint from the queue.
"""

# Default acceptance radius for the UAV in meters
DEFAULT_ACCEPTANCE_RADIUS = 1.0

DEBUG_NAV = False


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        # Declare parameters
        self.declare_parameter('acceptance_radius', DEFAULT_ACCEPTANCE_RADIUS)

        # Get the acceptance radius for the UAV, which should be a float
        # between 1 and 20
        self.acceptance_radius = self.get_parameter('acceptance_radius').get_parameter_value().double_value
        if(not isinstance(self.acceptance_radius, float) or self.acceptance_radius < 1 or self.acceptance_radius > 20):
            self.get_logger().info(f"{self.get_name()}: Acceptance radius should be a float between 1 and 20")
            rclpy.shutdown()
            return

        self.get_logger().info(f"{self.get_name()}: AR: {self.acceptance_radius}")

        # Target goal for the UAV in GoToTarget mode
        self.robot_target = None

        # Create action client
        self.wp_action_client = ActionClient(self, WaypointMove, '/nimbus/pilot/set_waypoint')
        # Handle needed to cancel waypoint actions
        self.current_client_goal_handle = None

        # Track when we have completed a waypoint
        self.reached_waypoint = True
        self.dist_to_waypoint = 0
        self.dist_total = -1

        # Create the action server
        self.move_action_server = ActionServer(
            self,
            WaypointSequence,
            'navigator/set_route',
            execute_callback=self.position_execute_callback,
            cancel_callback=self.position_cancel_callback,
            goal_callback=self.position_goal_callback
        )
        self.get_logger().info("WaypointSequence action server started.")

        # We are good to go!
        self.get_logger().info(f"{self.get_name()}: Started")


    # --- Request Waypoint action from pilot --------------------------------
    def send_wp_goal(self, waypoint: int, tolerance: float):
        # Wait for the action server to come online
        self.wp_action_client.wait_for_server()

        # Create goal message
        goal_msg = WaypointMove.Goal()
        goal_msg.waypoint = waypoint
        goal_msg.tolerance = tolerance

        # Goal tracking
        self.reached_waypoint = False

        # Logging
        self.get_logger().info(f'Sending pilot waypoint {waypoint}, tolerance = {tolerance}')

        # Send goal asynchronously
        self._send_goal_future = self.wp_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.wp_feedback_callback
        )

        # Attach callback for request result
        self._send_goal_future.add_done_callback(self.wp_response_callback)


    # --- Waypoint request response -----------------------------------------
    def wp_response_callback(self, future):
        goal_handle = future.result()

        # Did the pilot reject the goal?
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint rejected')
            self.reached_waypoint = True
            return

        self.get_logger().info('Waypoint accepted')

        # Store the action handle (used to cancel action, if needed)
        self.current_client_goal_handle = goal_handle

        # Attach result callback
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.wp_result_callback)


    # --- Waypoint request feedback -----------------------------------------
    def wp_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.dist_to_waypoint = feedback.distance_to_go
        if self.dist_total < 0:
            self.dist_total = feedback.distance_to_go
        
        # Logging
        if DEBUG_NAV:
            self.get_logger().info(f'Distance to go: {feedback.distance_to_go:.2f} m')


    # --- Waypoint request final result -------------------------------------
    def wp_result_callback(self, future):
        result = future.result().result
        # Regardless of the result, mark that the action ended
        self.reached_waypoint = True
        self.dist_total = -1

        self.get_logger().info(f'Waypoint action result: {result.success}')


    # --- Position goal validation ------------------------------------------
    def position_goal_callback(self, goal_request):
        self.get_logger().info(f"Received WaypointSequence request: {goal_request.waypoints}, tolerance: {goal_request.tolerance}")

        # Accept or reject the incoming goal
        return rclpy.action.GoalResponse.ACCEPT


    # --- Position cancel handling ------------------------------------------
    def position_cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT


    # --- Main position-move execution --------------------------------------
    async def position_execute_callback(self, goal_handle):
        self.get_logger().info("Executing WaypointSequence action...")

        # Get the waypoint sequence that we want to move to (type PointStamped)
        quad_route = goal_handle.request.waypoints

        # Waypoint tolerance should be no less than navigator's acceptance radius
        waypoint_tolerance = max(goal_handle.request.tolerance, self.acceptance_radius)

        # Verify that the sequence isn't empyt...
        if len(quad_route) == 0:
            # No waypoint given!
            self.get_logger().info("Received empty waypoint sequence!")

            goal_handle.succeed()
            result = WaypointSequence.Result()
            result.success = True
            return result

        # Print the goal
        self.get_logger().info(
            f"Received WaypointSequence w/ {len(quad_route)} stops --> {quad_route[-1]}"
        )

        # Send first waypoint
        self.send_wp_goal(quad_route[0], waypoint_tolerance)

        feedback_msg = WaypointSequence.Feedback()
        work_complete = 0
        toal_work = len(quad_route)

        # Run action-loop
        while rclpy.ok():
            # Did the action get cancelled..?
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling current waypoint move...")
                # Is there an active sub-action running?
                if self.current_client_goal_handle is not None:
                    # Request cancellation of the sub-action
                    future = self.current_client_goal_handle.cancel_goal_async()
                # Cancel this goal
                goal_handle.canceled()
                return WaypointSequence.Result(success=False)

            # Did we reach the current waypoint?
            if self.reached_waypoint:
                # Update waypoint
                quad_route.pop(0)
                work_complete += 1
                # Did we hit the end of the waypoint queue?
                if len(quad_route) < 1:
                    # Done, break!
                    break
                else:
                    # Request next waypoint
                    self.send_wp_goal(quad_route[0], waypoint_tolerance)

            # Update progress
            if self.dist_total > 0:
                # Add in mid-waypoint progress
                progress = work_complete/toal_work + ((1/toal_work)*(1.0-self.dist_to_waypoint/self.dist_total))
            else:
                # Just report the waypoint progress
                progress = work_complete/toal_work
            # Report progress
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)

            if DEBUG_NAV:
                self.get_logger().info(f"Progress: {progress:.2f}")

            # Spin ROS
            rclpy.spin_once(self)

        # Done
        result = WaypointSequence.Result()
        result.success = True

        self.get_logger().info("WaypointSequence complete.")
        goal_handle.succeed()
        return result



def main(args=None):
    rclpy.init(args=args)

    nav_node = Navigator()

    print(f"\tSpinning node...")

    rclpy.spin(nav_node)

    nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
