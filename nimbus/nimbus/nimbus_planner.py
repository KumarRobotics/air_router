#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from router_interfaces.action import WaypointMove, WaypointSequence

# # --- MOCK IMPORTS FOR DEMONSTRATION (Delete these in your actual code) ---
# # Assuming these exist in your router_interfaces package based on your description
# class WaypointMove:
#     class Goal: pass
#     class Result: pass
#     class Feedback: pass
#     class Impl: pass 
#     # Just a dummy class structure for the linter

# class WaypointSequence:
#     class Goal: pass
#     class Result: pass
#     class Feedback: pass
# # -------------------------------------------------------------------------


class NimbusPlanner(Node):

    def __init__(self):
        super().__init__('planner')

        # Create a ReentrantCallbackGroup to allow the server and client 
        # to process callbacks concurrently (essential for cancellation to work)
        self._cb_group = ReentrantCallbackGroup()

        # 1. The Action Server: Accepts a request to go to a specific single waypoint
        self._action_server = ActionServer(
            self,
            WaypointMove,
            'planner/move_to_waypoint', # Topic name for this planner
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )

        # 2. The Action Client: Sends the sequence of waypoints to the navigator
        self._navigator_client = ActionClient(
            self,
            WaypointSequence,
            '/nimbus/navigator/set_route',
            callback_group=self._cb_group
        )

        # Track the active handle for the client so we can cancel it if needed
        self._current_navigator_goal_handle = None

        self.get_logger().info("Nimbus Planner is ready.")

    def goal_callback(self, goal_request):
        """Accept or reject incoming goals."""
        self.get_logger().info(f"Planner received request for Waypoint: {goal_request.waypoint}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info("Planner received cancel request.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Main execution logic:
        1. Calculate route.
        2. Send route to Navigator.
        3. Monitor result and handle cancellation.
        """
        self.get_logger().info("Executing plan...")

        # Extract request data
        target_waypoint = goal_handle.request.waypoint
        tolerance = goal_handle.request.tolerance

        # --- STEP 1: Generate the Route ---
        # Call your custom logic to get the list of waypoints
        route_sequence = self.calculate_route(target_waypoint)
        
        if not route_sequence:
            self.get_logger().warn(f"Could not find a route to waypoint {target_waypoint}")
            goal_handle.abort()
            result = WaypointMove.Result()
            result.success = False
            return result
        
        self.get_logger().info(f"Generated route: {route_sequence}")

        # --- STEP 2: Send to Navigator ---
        if not self._navigator_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigator action server not available!")
            goal_handle.abort()
            return WaypointMove.Result(success=False)

        # Prepare the goal for the sub-action
        nav_goal = WaypointSequence.Goal()
        nav_goal.waypoints = route_sequence
        nav_goal.tolerance = tolerance

        # Send the goal asynchronously
        self.get_logger().info("Sending sequence to Navigator...")
        send_goal_future = self._navigator_client.send_goal_async(
            nav_goal,
            feedback_callback=self.navigator_feedback_callback
        )

        # Wait for the goal to be accepted/rejected
        # We use await here because we are in an async callback
        nav_goal_handle = await send_goal_future

        if not nav_goal_handle.accepted:
            self.get_logger().error("Navigator rejected the route.")
            goal_handle.abort()
            return WaypointMove.Result(success=False)

        # Store the handle so we can cancel it later if necessary
        self._current_navigator_goal_handle = nav_goal_handle

        # --- STEP 3: Wait for Result & Handle Cancellation ---
        result_future = nav_goal_handle.get_result_async()

        # We loop checking for cancellation while waiting for the result
        while not result_future.done():
            # Check if the PARENT action (Planner) has been canceled
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Planner canceling Navigator goal...")
                
                # Cancel the CHILD action (Navigator)
                # We do not await this, just fire and forget usually, or await if strictly needed
                await nav_goal_handle.cancel_goal_async()
                
                goal_handle.canceled()
                self._current_navigator_goal_handle = None
                return WaypointMove.Result(success=False)
            
            # Use a small sleep or await a short timer to prevent busy-looping
            # This allows the executor to process feedback callbacks
            try:
                # Wait for the future for a tiny amount of time
                # Note: 'asyncio.wait_for' is an alternative, but strict rclpy standard 
                # often relies on the executor. Here we just await the future with a check.
                # Since we can't easily "await with timeout" on a Future in pure rclpy 
                # without external libs, checking .done() in a loop with a small sleep is common pattern.
                import asyncio
                await asyncio.sleep(0.1) 
            except Exception:
                pass

        # --- STEP 4: Process Final Result ---
        nav_result = result_future.result()
        self._current_navigator_goal_handle = None
        
        result = WaypointMove.Result()
        
        status = nav_result.status
        # Check standard ROS 2 action status codes (4 = SUCCEEDED)
        if status == 4: # STATUS_SUCCEEDED
            self.get_logger().info("Navigator finished successfully.")
            result.success = True
            goal_handle.succeed()
        else:
            self.get_logger().warn(f"Navigator failed or was canceled with status: {status}")
            result.success = False
            # If the child failed, we abort the parent
            goal_handle.abort()

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

    def calculate_route(self, target_waypoint):
        """
        YOUR LOGIC HERE.
        Returns a list of integers (waypoints).
        """
        # Placeholder: just return a simple list ending in the target
        self.get_logger().info(f"Calculating route to {target_waypoint}...")
        
        # Example logic: [1, 2, ..., target]
        # Only for demonstration
        return [10, 20, target_waypoint]

def main(args=None):
    rclpy.init(args=args)
    node = NimbusPlanner()
    
    # Use MultiThreadedExecutor to ensure reentrant callbacks work smoothly
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    
    rclpy.spin(node, executor=executor)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()