#!/usr/bin/env python3
import os
import pdb
import threading
from enum import Enum, auto
from os import wait

import cv2
import numpy as np
# import rospy
import utm
import yaml
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from mavros_msgs.srv import SetMode, WaypointSetCurrent
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import CancelResponse

from nimbus import route_planner
from router_interfaces.msg import Goal
from router_interfaces.action import WaypointMove
from router_interfaces.action import PositionMove

"""Navigator:
This node is responsible for navigating the UAV along the waypoints. It
receives a working mode from the state machine("go to robot", "explore") and
creates a plan for performing the action using the router.
"""

# Default acceptance radius for the UAV in meters
DEFAULT_ACCEPTANCE_RADIUS = 3
DEFAULT_MAX_EDGE_LENGTH = 100

DEBUG_NAV = False


# def cv_to_ros(img):
#     image_msg = Image()
#     image_msg.encoding = "bgr8"
#     image_msg.height = img.shape[0]
#     image_msg.width = img.shape[1]
#     image_msg.step = image_msg.width * 3
#     image_msg.data = img.tobytes()
#     return image_msg


class Navigator(Node):
    # class Mode(Enum):
    #     init = auto()
    #     explore = auto()
    #     explore_end = auto()
    #     go_to_target = auto()
    #     go_to_target_end = auto()
    #     transition = auto()

    def __init__(self):
        super().__init__('nimbus_navigator')

        # Declare parameters
        self.declare_parameter('acceptance_radius', DEFAULT_ACCEPTANCE_RADIUS)
        # self.declare_parameter('sim', True)
        self.declare_parameter('max_edge_length', DEFAULT_MAX_EDGE_LENGTH)
        # self.declare_parameter('config_path', value="PARAMETER NOT SET")

        # Get the acceptance radius for the UAV, which should be an integer
        # between 1 and 20
        self.acceptance_radius = self.get_parameter('acceptance_radius').get_parameter_value().integer_value

        if(not isinstance(self.acceptance_radius, int) or self.acceptance_radius < 1 or self.acceptance_radius > 20):
            self.get_logger().info(f"{self.get_name()}: Acceptance radius should be an integer between 1 and 20")
            rclpy.shutdown()
            return

        # # Are we in simulator mode?
        # self.sim = self.get_parameter('sim').get_parameter_value().bool_value
        
        # Get the edge length for route planner
        self.max_edge_length = self.get_parameter('max_edge_length').get_parameter_value().integer_value
        if(not isinstance(self.max_edge_length, int) or self.max_edge_length < 1 or self.max_edge_length > 500):
            self.get_logger().error(f"{self.get_name()}: Max edge length should be an integer between 1 and 500")
            rclpy.shutdown()
            return

        # # Does the map config file exist?
        # self.config_file = self.get_parameter('config_path').get_parameter_value().string_value
        # if not os.path.exists(self.config_file):
        #     self.get_logger().error(f"Map config file does not exist")
        #     rclpy.shutdown()
        #     return
        # assert isinstance(self.sim, bool)
        # self.get_logger().info(f"{self.get_name()}: Config File: {self.config_file}")
        self.get_logger().info(f"{self.get_name()}: Max edge length: {self.max_edge_length}")
        # self.get_logger().info(f"{self.get_name()}: Sim: {self.sim}")
        self.get_logger().info(f"{self.get_name()}: AR: {self.acceptance_radius}")

        # TODO # Create a path planner object
        # self.planner = route_planner.Path_planner(self.config_file,
        #                                           self.max_edge_length, self)

        # # # Initially, the navigator is in the init mode. We will wait for an
        # # # order from the state machine
        # # # Mode gets published so other nodes can use it
        # # self.mode = None
        # # self.mode_lock = threading.Lock()
        # # self.mode_pub = self.create_publisher(String, 'air_router/navigator/state', 10)


        # # List of waypoints for exploration
        # self.waypoint_list = list(self.planner.mission.waypoints.keys())
        # self.explore_target_waypt = self.waypoint_list.copy()
        # self.end_waypt = self.explore_target_waypt[-1]

        # # UAV pose keeps the pose of the UAV in standard coordinates:
        # self.uav_pose = None

        # Target goal for the UAV in GoToTarget mode
        self.robot_target = None

        # # Create threads for the different modes
        # self.stop_exploration = threading.Event()
        # self.stop_go_to_target = threading.Event()
        # self.explore_thread = None

        # # Best-Effort QoS
        # be_qos = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )

        # # Create a subscriber for the UAV position. Use GPS input
        # self.pose_sub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.gps_callback, be_qos)

        # # Publish the goal for the UAV
        # self.set_cur_wp = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')
        # while not self.set_cur_wp.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.get_logger().info('Waypoint service up')

        # # Create subscribers _after loading services_ for the state machine
        # # topics: goal and coordinates
        # self.create_subscription(Goal, 'air_router/goal', self.goal_callback, 1)

        # # Create the visualization topic to debug the navigator
        # self.vis_pub = self.create_publisher(Image, 'air_router/navigator/viz', 1)

        # self.get_logger().info(f"{self.get_name()}: Waiting for UAV pose")
        # while not rclpy.ok() and self.uav_pose is None:
        #     self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # # Avoid race condition with state machine
        # self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))

        # Create action client
        self.wp_action_client = ActionClient(self, WaypointMove, '/nimbus/pilot/set_waypoint')

        # Track when we have completed a waypoint
        self.reached_waypoint = True
        self.dist_to_waypoint = 0
        self.dist_total = -1

        # Create the action server
        self.move_action_server = ActionServer(
            self,
            PositionMove,
            'navigator/set_position',
            execute_callback=self.position_execute_callback,
            cancel_callback=self.position_cancel_callback,
            goal_callback=self.position_goal_callback
        )

        # We are good to go!
        # self.set_mode(self.Mode.init)
        self.get_logger().info(f"{self.get_name()}: Started")
        self.get_logger().info("PositionMove action server started.")


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
        self.get_logger().info(f"Received PositionMove request: {goal_request.action}")

        # Accept or reject the incoming goal
        return rclpy.action.GoalResponse.ACCEPT


    # --- Position cancel handling ------------------------------------------
    def position_cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT


    # --- Main position-move execution --------------------------------------
    async def position_execute_callback(self, goal_handle):
        self.get_logger().info("Executing PositionMove action...")

        # Get the point that we want to move to (type PointStamped)
        point = goal_handle.request.goal

        # Stupid, chat-suggested syntax for doing the above...
        # goal_msg: PositionMove.Goal = goal_handle.request
        # point: PointStamped = goal_msg.goal

        # Example: Print the goal
        self.get_logger().info(
            f"Goal point received: frame={point.header.frame_id}, "
            f"x={point.point.x}, y={point.point.y}, z={point.point.z}"
        )

        feedback_msg = PositionMove.Feedback()

        # Send first waypoint
        # TODO: this should be determined from the route planner
        quad_route = [[5, 2.0], [4, 2.0], [3, 2.0]]
        self.send_wp_goal(quad_route[0][0], quad_route[0][1])

        work_complete = 0
        toal_work = len(quad_route)

        rate = self.create_rate(10)

        # Run action-loop
        while rclpy.ok():
            # Did the action get cancelled..?
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("PositionMove canceled.")
                return PositionMove.Result(success=False)
            
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
                    self.send_wp_goal(quad_route[0][0], quad_route[0][1])

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
        result = PositionMove.Result()
        result.success = True

        self.get_logger().info("PositionMove succeeded.")
        goal_handle.succeed()
        return result


    # def set_mode(self, mode):
    #     self.mode_lock.acquire()
    #     self.mode = mode
    #     self.mode_lock.release()
    #     self.get_logger().info(f"{self.get_name()}: Mode: {mode.name}")
    #     # Publish the mode
    #     msg = String()
    #     msg.data = mode.name
    #     self.mode_pub.publish(msg)

    # def goal_callback(self, data):
    #     # Check that goal is either "explore" or "go to robot"
    #     assert data.action in ["explore", "go to robot"]
    #     if self.mode is self.Mode.init and data.action == "explore":
    #         # initial explore
    #         self.explore_thread = self.ExplorationThread(self, self.stop_exploration)
    #         self.explore_thread.daemon = True
    #         self.set_mode(self.Mode.explore)
    #         self.explore_thread.start()
    #     elif self.mode is self.Mode.init and data.action == "go to robot":
    #         self.robot_target = data.goal.point
    #         self.goto_target_thread = self.GoToTargetThread(
    #             self, self.stop_go_to_target
    #         )
    #         self.goto_target_thread.daemon = True
    #         self.set_mode(self.Mode.go_to_target)
    #         self.goto_target_thread.start()

    #     elif self.mode == self.Mode.explore and data.action == "explore":
    #         return
    #     elif self.mode == self.Mode.go_to_target and data.action == "go to robot":
    #         # Signal the go to robot thread to stop
    #         self.stop_go_to_target.set()
    #         self.goto_target_thread.join()
    #         # Go find the robot
    #         self.robot_target = data.goal.point
    #         self.goto_target_thread = self.GoToTargetThread(
    #             self, self.stop_go_to_target
    #         )
    #         self.goto_target_thread.daemon = True
    #         self.set_mode(self.Mode.go_to_target)
    #         self.goto_target_thread.start()
    #     elif (
    #         self.mode == self.Mode.explore
    #         or self.mode == self.Mode.explore_end
    #         or self.mode == self.Mode.go_to_target_end
    #         and data.action == "go to robot"
    #     ):
    #         # Signal the exploration thread to stop, only if we started
    #         # exploring before
    #         if self.explore_thread is not None:
    #             self.stop_exploration.set()
    #             self.explore_thread.join()
    #             self.explore_thread = None
    #         # Go find the robot
    #         self.robot_target = data.goal.point
    #         self.goto_target_thread = self.GoToTargetThread(
    #             self, self.stop_go_to_target
    #         )
    #         self.goto_target_thread.daemon = True
    #         self.set_mode(self.Mode.go_to_target)
    #         self.goto_target_thread.start()
    #     elif (
    #         self.mode == self.Mode.go_to_target
    #         or self.mode == self.Mode.go_to_target_end
    #         and data.action == "explore"
    #     ):
    #         # Signal the go to robot thread to stop
    #         self.stop_go_to_target.set()
    #         self.goto_target_thread.join()
    #         self.set_mode(self.Mode.transition)
    #         # Go to the last exploration position
    #         p = self.planner.mission.waypoints[self.explore_target_waypt[0]]
    #         alt = 40
    #         self.robot_target = Point(p[0], p[1], alt)
    #         self.goto_target_thread = self.GoToTargetThread(
    #             self, self.stop_go_to_target
    #         )
    #         self.goto_target_thread.daemon = True
    #         self.goto_target_thread.start()
    #         self.goto_target_thread.join()
    #         # Resume exploration
    #         self.explore_thread = self.ExplorationThread(self, self.stop_exploration)
    #         self.set_mode(self.Mode.explore)
    #         self.explore_thread.daemon = True
    #         self.explore_thread.start()
    #     else:
    #         # unknown transition. Die
    #         self.get_logger().error("Unknown transition from %s to %s", self.mode, data.action)
    #         rclpy.shutdown()

    # def pose_callback(self, data):
    #     self.uav_pose = data

    # def gps_callback(self, data):
    #     # Convert the GPS coordinates to the map frame
    #     lat = data.latitude
    #     lon = data.longitude
    #     x, y = np.array(utm.from_latlon(lat, lon)[:2]) - np.array(utm.from_latlon(self.planner.origin[0], self.planner.origin[1])[:2])
    #     pose = PoseStamped()
    #     pose.header.frame_id = "quad"
    #     pose.header.stamp = self.get_clock().now().to_msg()
    #     pose.pose.position.x = x
    #     pose.pose.position.y = y
    #     self.uav_pose = pose

    # def callback_waypoint_uav(self, future: Future):
    #     response = future.result()
    #     if response is not None:
    #         self.get_logger().info(f"Waypoint service response: {response.success}")
    #     else:
    #         self.get_logger().warn("Waypoint service response was None.")

    # def send_waypoint_uav(self, target_wpt):
    #     self.get_logger().info(f"Asked to move to wp {target_wpt}")
    #     wp_req = WaypointSetCurrent.Request()
    #     wp_req.wp_seq = target_wpt
    #     # Call the mavros service to set the current waypoint
    #     try:
    #         self.future = self.set_cur_wp.call_async(wp_req)
    #         self.future.add_done_callback(self.callback_waypoint_uav)
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {e}")

    # def arrived_at_waypoint(self, waypoint):
    #     if self.uav_pose is not None:
    #         wp = self.planner.mission.waypoints[waypoint]
    #         curr = np.array(
    #             [self.uav_pose.pose.position.x, self.uav_pose.pose.position.y]
    #         )
    #         # self.get_logger().info(f"Current position: {curr}, target: {wp} ({waypoint})")
    #         if np.linalg.norm(curr - wp) < self.acceptance_radius:
    #             return True
    #     else:
    #         self.get_logger().warn(f"{self.get_name()}: UAV pose not received yet")
    #     return False

    # class ExplorationThread(threading.Thread):
    #     def __init__(self, outer, stop_event):
    #         super().__init__()
    #         self.stop_event = stop_event
    #         self.outer = outer

    #     def run(self):
    #         self.stop_event.clear()
    #         # rate = rospy.Rate(10)
    #         # ROS2 equivalent of rospy.Rate(10)
    #         rate = self.outer.create_rate(10)
    #         rclpy.node.get_logger("navigator:ExplorationThread").info(f"Exploration - Start")
    #         while rclpy.ok() and not self.stop_event.is_set():
    #             # Get the top element on the list as target waypoint
    #             target = self.outer.explore_target_waypt[0]
    #             # Send target waypoint to the UAV and check if we arrived. If we
    #             # arrived, send the next waypoint
    #             self.outer.send_waypoint_uav(target)

    #             # Visualize the waypoint sent
    #             img = self.outer.planner.display_points(
    #                 get_image=True, waypoints=True, noFly=True
    #             )
    #             target_pos = self.outer.planner.mission.waypoints[target]
    #             target_px = self.outer.planner.scale_points(
    #                 target_pos[0], target_pos[1]
    #             )
    #             img = cv2.circle(img, tuple(target_px), 10, (0, 0, 255), 2)
    #             self.outer.vis_pub.publish(cv_to_ros(img))

    #             rclpy.node.get_logger("navigator:ExplorationThread").info(
    #                 f"Exploration - Going to waypoint {target}"
    #             )
    #             while (
    #                 not self.outer.arrived_at_waypoint(target)
    #                 and rclpy.ok()
    #                 and not self.stop_event.is_set()
    #             ):
    #                 # Wait to arrive at the waypoint
    #                 rate.sleep()
    #             if self.outer.arrived_at_waypoint(target):
    #                 if target == self.outer.end_waypt:
    #                     self.outer.set_mode(self.outer.Mode.explore_end)
    #                     return
    #                 self.outer.explore_target_waypt = self.outer.explore_target_waypt[
    #                     1:
    #                 ] + [target]

    # class GoToTargetThread(threading.Thread):
    #     def __init__(self, outer, stop_event):
    #         super().__init__()
    #         self.stop_event = stop_event
    #         self.outer = outer

    #     def run(self):
    #         self.stop_event.clear()
    #         rate = self.outer.create_rate(10)

    #         # rclpy.node.get_logger("navigator:ExplorationThread").info(f"GoToTarget - Start")

    #         # Wait until we have a valid robot pose
    #         while (
    #             rclpy.ok()
    #             and not self.stop_event.is_set()
    #             and self.outer.uav_pose is None
    #         ):
    #             rate.sleep()

    #         # Get the current position of the robot and target
    #         pos = self.outer.uav_pose.pose.position

    #         robot_target = self.outer.robot_target

    #         # Get the trajectory from the current position to the target
    #         route = self.outer.planner.planRoute(
    #             [pos.x, pos.y], [robot_target.x, robot_target.y]
    #         )
    #         if route is None:
    #             rclpy.node.get_logger("navigator:ExplorationThread").error(f"GoToTarget - Could not find route.")
    #             return

    #         if len(route) == 0:
    #             rclpy.node.get_logger("navigator:ExplorationThread").info(
    #                 f"GoToTarget - Already at the target."
    #             )
    #             return

    #         # Check if we are already at the first waypoint
    #         if self.outer.arrived_at_waypoint(route[0]):
    #             if len(route) == 1:
    #                 rclpy.node.get_logger("navigator:ExplorationThread").info(
    #                     f"GoToTarget - Already at the target."
    #                 )
    #                 return
    #             else:
    #                 route = route[1:]

    #         rclpy.node.get_logger("navigator:ExplorationThread").info(f"GoToTarget - Route: {route}")

    #         while rclpy.ok() and not self.stop_event.is_set():
    #             # Get the top element of the route
    #             target = route.pop(0)
    #             rclpy.node.get_logger("navigator:ExplorationThread").info(
    #                 f"GoToTarget - Going to waypoint {target}"
    #             )
    #             # Send target waypoint to the UAV and check if we arrived. If we
    #             # arrived, send the next waypoint
    #             self.outer.send_waypoint_uav(target)

    #             decimator = 0
    #             while (
    #                 not self.outer.arrived_at_waypoint(target)
    #                 and rclpy.ok()
    #                 and not self.stop_event.is_set()
    #             ):
    #                 # Wait to arrive at the waypoint and publish images at 0.5
    #                 # hz
    #                 if decimator % 20 == 0:
    #                     # Visualize the waypoint sent
    #                     img = self.outer.planner.display_points(
    #                         get_image=True, routes=True, plan=True, noFly=True
    #                     )
    #                     target_pos = self.outer.planner.mission.waypoints[target]
    #                     target_px = self.outer.planner.scale_points(
    #                         target_pos[0], target_pos[1]
    #                     )
    #                     robot_target_px = self.outer.planner.scale_points(
    #                         robot_target.x, robot_target.y
    #                     )
    #                     uav_pos = self.outer.uav_pose.pose.position
    #                     uav_pos_px = self.outer.planner.scale_points(
    #                         uav_pos.x, uav_pos.y
    #                     )
    #                     img = cv2.circle(img, tuple(target_px), 10, (0, 0, 255), 2)
    #                     img = cv2.circle(
    #                         img, tuple(robot_target_px), 10, (0, 255, 0), 2
    #                     )
    #                     img = cv2.circle(img, tuple(uav_pos_px), 8, (0, 150, 255), -1)
    #                     self.outer.vis_pub.publish(cv_to_ros(img))
    #                     decimator = 0
    #                 decimator += 1
    #                 rate.sleep()

    #             # Check if we made it to the end
    #             if len(route) == 0 and self.outer.arrived_at_waypoint(target):
    #                 self.outer.set_mode(self.outer.Mode.go_to_target_end)
    #                 rclpy.node.get_logger("navigator:ExplorationThread").info(f"GoToTarget: reached goal")
    #                 return


def main(args=None):
    rclpy.init(args=args)

    nav_node = Navigator()

    print(f"\tSpinning node...")

    rclpy.spin(nav_node)

    nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
