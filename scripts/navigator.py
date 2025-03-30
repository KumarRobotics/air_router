#!/usr/bin/env python3
import os
import pdb
import threading
from enum import Enum, auto
from os import wait

import cv2
import numpy as np
import rospy
import utm
import yaml
from air_router.msg import Goal
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from mavros_msgs.srv import SetMode, WaypointSetCurrent
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String

import route_planner

"""Navigator:
This node is responsible for navigating the UAV along the waypoints. It
receives a working mode from the state machine("go to robot", "explore") and
creates a plan for performing the action using the router.
"""

# Default acceptance radius for the UAV in meters
DEFAULT_ACCEPTANCE_RADIUS = 3
DEFAULT_MAX_EDGE_LENGTH = 100


def cv_to_ros(img):
    image_msg = Image()
    image_msg.encoding = "bgr8"
    image_msg.height = img.shape[0]
    image_msg.width = img.shape[1]
    image_msg.step = image_msg.width * 3
    image_msg.data = img.tobytes()
    return image_msg


class Navigator:
    # Modes for the navigator:
    # Init: The navigator is waiting for a command from the state machine
    # Explore: the navigator is exploring the map, going sequentially through
    #          the waypoints
    # GoToTarget: the navigator is going to the target waypoint
    # Transition: similar to GoToTarget, but we are not going to a target
    #             waypoint, we are transitioning from GoToTarget to Explore
    class Mode(Enum):
        init = auto()
        explore = auto()
        explore_end = auto()
        go_to_target = auto()
        go_to_target_end = auto()
        transition = auto()

    def __init__(self):
        rospy.init_node("navigator", anonymous=False)

        # Get the acceptance radius for the UAV, which should be an integer
        # between 1 and 20
        self.acceptance_radius = rospy.get_param(
            "~acceptance_radius", DEFAULT_ACCEPTANCE_RADIUS
        )
        if (
            not isinstance(self.acceptance_radius, int)
            or self.acceptance_radius < 1
            or self.acceptance_radius > 20
        ):
            rospy.logerr(
                f"{rospy.get_name()}: \
                    Acceptance radius should be an integer between 1 and 20"
            )
            rospy.signal_shutdown("Acceptance radius error")
            return

        # Are we in simulator mode?
        self.sim = rospy.get_param("~sim", True)

        # Get the path to the map file
        if not rospy.has_param("~world_config_path"):
            rospy.logfatal(f"{rospy.get_name()}: world_config_path is not set")
            rospy.signal_shutdown("World_config_path is not set")
            return
        self.world_config_path = rospy.get_param("~world_config_path")
        rospy.loginfo(
            f"{rospy.get_name()}: World config path: {self.world_config_path}"
        )
        # Get the base path from the world_config_path
        path = os.path.dirname(self.world_config_path)
        with open(self.world_config_path, "r") as f:
            world_config = yaml.safe_load(f)
        self.map = os.path.join(path, world_config["map"])

        # Get the edge length for route planner
        self.max_edge_length = rospy.get_param(
            "~max_edge_length", DEFAULT_MAX_EDGE_LENGTH
        )
        if (
            not isinstance(self.max_edge_length, int)
            or self.max_edge_length < 1
            or self.max_edge_length > 500
        ):
            rospy.logerr(
                f"{rospy.get_name()}: \
                    Max edge length should be an integer between 1 and 500"
            )

        # Does the map config file exist?
        if not os.path.exists(self.map):
            rospy.logfatal(f"{rospy.get_name()}: Map config file does not exist")
            rospy.signal_shutdown("Map config file does not exist")
            return
        assert isinstance(self.sim, bool)
        rospy.loginfo(f"{rospy.get_name()}: Map: {self.map}")
        rospy.loginfo(f"{rospy.get_name()}: Max edge length: {self.max_edge_length}")
        rospy.loginfo(f"{rospy.get_name()}: Sim: {self.sim}")
        rospy.loginfo(f"{rospy.get_name()}: AR: {self.acceptance_radius}")

        # Create a path planner object
        self.planner = route_planner.Path_planner(self.map, self.max_edge_length)

        # Initially, the navigator is in the init mode. We will wait for an
        # order from the state machine
        # Mode gets published so other nodes can use it
        self.mode = None
        self.mode_lock = threading.Lock()
        self.mode_pub = rospy.Publisher(
            "air_router/navigator/state", String, queue_size=10
        )

        # List of waypoints for exploration
        self.waypoint_list = list(self.planner.mission.waypoints.keys())
        self.explore_target_waypt = self.waypoint_list.copy()
        self.end_waypt = self.explore_target_waypt[-1]

        # UAV pose keeps the pose of the UAV in standard coordinates:
        self.uav_pose = None

        # Counters for ROS msgs
        self.uav_goal_seq = 0
        self.uav_pose_seq = 0

        # Target goal for the UAV in GoToTarget mode
        self.robot_target = None

        # Create threads for the different modes
        self.stop_exploration = threading.Event()
        self.stop_go_to_target = threading.Event()
        self.explore_thread = None

        # Create a subscriber for the UAV position. This is for the simulator.
        # For the real world, we will use the GPS input here
        if self.sim:
            rospy.Subscriber(
                "/unity_ros/quadrotor/TrueState/pose", PoseStamped, self.pose_callback
            )
        else:
            rospy.Subscriber(
                "mavros/global_position/global", NavSatFix, self.gps_callback
            )

        # Publish the goal for the UAV. For simulation, we will just publish a
        # goal, for the real world, we will use the mavros interface
        if self.sim:
            self.uav_goal = rospy.Publisher("goal", PointStamped, queue_size=10)
        else:
            # Service proxy for /mavros/misssion/set_current
            rospy.wait_for_service("mavros/mission/set_current")
            self.set_cur_wp = rospy.ServiceProxy(
                "mavros/mission/set_current", WaypointSetCurrent
            )

        # Create subscribers _after loading services_ for the state machine
        # topics: goal and coordinates
        rospy.Subscriber("air_router/goal", Goal, self.goal_callback)

        # Create the visualization topic to debug the navigator
        self.vis_pub = rospy.Publisher("air_router/navigator/viz", Image, queue_size=1)

        rospy.loginfo(f"{rospy.get_name()}: Waiting for UAV pose")
        while not rospy.is_shutdown() and self.uav_pose is None:
            rospy.sleep(0.1)

        # Avoid race condition with state machine
        rospy.sleep(1)

        # We are good to go!
        self.set_mode(self.Mode.init)
        rospy.loginfo(f"{rospy.get_name()}: Started")

    def set_mode(self, mode):
        self.mode_lock.acquire()
        self.mode = mode
        self.mode_lock.release()
        rospy.loginfo(f"{rospy.get_name()}: Mode: {mode.name}")
        self.mode_pub.publish(mode.name)

    def goal_callback(self, data):
        # Check that goal is either "explore" or "go to robot"
        assert data.action in ["explore", "go to robot"]
        if self.mode is self.Mode.init and data.action == "explore":
            # initial explore
            self.explore_thread = self.ExplorationThread(self, self.stop_exploration)
            self.explore_thread.daemon = True
            self.set_mode(self.Mode.explore)
            self.explore_thread.start()
        elif self.mode is self.Mode.init and data.action == "go to robot":
            self.robot_target = data.goal.point
            self.goto_target_thread = self.GoToTargetThread(
                self, self.stop_go_to_target
            )
            self.goto_target_thread.daemon = True
            self.set_mode(self.Mode.go_to_target)
            self.goto_target_thread.start()

        elif self.mode == self.Mode.explore and data.action == "explore":
            return
        elif self.mode == self.Mode.go_to_target and data.action == "go to robot":
            # Signal the go to robot thread to stop
            self.stop_go_to_target.set()
            self.goto_target_thread.join()
            # Go find the robot
            self.robot_target = data.goal.point
            self.goto_target_thread = self.GoToTargetThread(
                self, self.stop_go_to_target
            )
            self.goto_target_thread.daemon = True
            self.set_mode(self.Mode.go_to_target)
            self.goto_target_thread.start()
        elif (
            self.mode == self.Mode.explore
            or self.mode == self.Mode.explore_end
            or self.mode == self.Mode.go_to_target_end
            and data.action == "go to robot"
        ):
            # Signal the exploration thread to stop, only if we started
            # exploring before
            if self.explore_thread is not None:
                self.stop_exploration.set()
                self.explore_thread.join()
                self.explore_thread = None
            # Go find the robot
            self.robot_target = data.goal.point
            self.goto_target_thread = self.GoToTargetThread(
                self, self.stop_go_to_target
            )
            self.goto_target_thread.daemon = True
            self.set_mode(self.Mode.go_to_target)
            self.goto_target_thread.start()
        elif (
            self.mode == self.Mode.go_to_target
            or self.mode == self.Mode.go_to_target_end
            and data.action == "explore"
        ):
            # Signal the go to robot thread to stop
            self.stop_go_to_target.set()
            self.goto_target_thread.join()
            self.set_mode(self.Mode.transition)
            # Go to the last exploration position
            p = self.planner.mission.waypoints[self.explore_target_waypt[0]]
            if self.sim:
                alt = self.planner.mission.altitude[self.explore_target_waypt[0]]
            else:
                alt = 40
            self.robot_target = Point(p[0], p[1], alt)
            self.goto_target_thread = self.GoToTargetThread(
                self, self.stop_go_to_target
            )
            self.goto_target_thread.daemon = True
            self.goto_target_thread.start()
            self.goto_target_thread.join()
            # Resume exploration
            self.explore_thread = self.ExplorationThread(self, self.stop_exploration)
            self.set_mode(self.Mode.explore)
            self.explore_thread.daemon = True
            self.explore_thread.start()
        else:
            # unknown transition. Die
            rospy.logerr("Unknown transition from %s to %s", self.mode, data.action)
            rospy.signal_shutdown("Shutting down navigator")

    def pose_callback(self, data):
        self.uav_pose = data

    def gps_callback(self, data):
        # Convert the GPS coordinates to the map frame
        lat = data.latitude
        lon = data.longitude
        x, y = utm.from_latlon(lat, lon)[0:2] - self.planner.origin
        pose = PoseStamped()
        pose.header.frame_id = "quad"
        pose.header.stamp = rospy.Time.now()
        pose.header.seq = self.uav_pose_seq
        self.uav_pose_seq += 1
        pose.pose.position.x = x
        pose.pose.position.y = y
        self.uav_pose = pose

    def send_waypoint_uav(self, target_wpt):
        # For simulation purposes, we will publish the target waypoint
        if self.sim:
            target = PointStamped()
            target.header.seq = self.uav_goal_seq
            self.uav_goal_seq += 1
            target.header.stamp = rospy.Time.now()
            waypoint = self.planner.mission.waypoints[target_wpt]
            alt = self.planner.mission.altitude[target_wpt]
            target.point.x = waypoint[0]
            target.point.y = waypoint[1]
            target.point.z = alt
            self.uav_goal.publish(target)
        else:
            # Call the mavros service to set the current waypoint
            try:
                self.set_cur_wp(target_wpt)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def arrived_at_waypoint(self, waypoint):
        if self.uav_pose is not None:
            wp = self.planner.mission.waypoints[waypoint]
            curr = np.array(
                [self.uav_pose.pose.position.x, self.uav_pose.pose.position.y]
            )
            # rospy.loginfo(f"Current position: {curr}, target: {wp}")
            if np.linalg.norm(curr - wp) < self.acceptance_radius:
                return True
        else:
            rospy.logwarn(f"{rospy.get_name()}: UAV pose not received yet")
        return False

    class ExplorationThread(threading.Thread):
        def __init__(self, outer, stop_event):
            super().__init__()
            self.stop_event = stop_event
            self.outer = outer

        def run(self):
            self.stop_event.clear()
            rate = rospy.Rate(10)
            # rospy.loginfo(f"{rospy.get_name()}: Exploration - Start")
            while not rospy.is_shutdown() and not self.stop_event.is_set():
                # Get the top element on the list as target waypoint
                target = self.outer.explore_target_waypt[0]
                # Send target waypoint to the UAV and check if we arrived. If we
                # arrived, send the next waypoint
                self.outer.send_waypoint_uav(target)

                # Visualize the waypoint sent
                img = self.outer.planner.display_points(
                    get_image=True, waypoints=True, noFly=True
                )
                target_pos = self.outer.planner.mission.waypoints[target]
                target_px = self.outer.planner.scale_points(
                    target_pos[0], target_pos[1]
                )
                img = cv2.circle(img, tuple(target_px), 10, (0, 0, 255), 2)
                self.outer.vis_pub.publish(cv_to_ros(img))

                rospy.loginfo(
                    f"{rospy.get_name()}: Exploration - Going to waypoint %s", target
                )
                while (
                    not self.outer.arrived_at_waypoint(target)
                    and not rospy.is_shutdown()
                    and not self.stop_event.is_set()
                ):
                    # Wait to arrive at the waypoint
                    rate.sleep()
                if self.outer.arrived_at_waypoint(target):
                    if target == self.outer.end_waypt:
                        self.outer.set_mode(self.outer.Mode.explore_end)
                        return
                    self.outer.explore_target_waypt = self.outer.explore_target_waypt[
                        1:
                    ] + [target]

    class GoToTargetThread(threading.Thread):
        def __init__(self, outer, stop_event):
            super().__init__()
            self.stop_event = stop_event
            self.outer = outer

        def run(self):
            self.stop_event.clear()
            rate = rospy.Rate(10)

            # rospy.loginfo(f"{rospy.get_name()}: GoToTarget - Start")

            # Wait until we have a valid robot pose
            while (
                not rospy.is_shutdown()
                and not self.stop_event.is_set()
                and self.outer.uav_pose is None
            ):
                rate.sleep()

            # Get the current position of the robot and target
            pos = self.outer.uav_pose.pose.position

            robot_target = self.outer.robot_target

            # Get the trajectory from the current position to the target
            route = self.outer.planner.planRoute(
                [pos.x, pos.y], [robot_target.x, robot_target.y]
            )
            if route is None:
                rospy.logerr(f"{rospy.get_name()}: GoToTarget - Could not find route.")
                return

            if len(route) == 0:
                rospy.loginfo(
                    f"{rospy.get_name()}: GoToTarget - Already at the target."
                )
                return

            # Check if we are already at the first waypoint
            if self.outer.arrived_at_waypoint(route[0]):
                if len(route) == 1:
                    rospy.loginfo(
                        f"{rospy.get_name()}: GoToTarget - Already at the target."
                    )
                    return
                else:
                    route = route[1:]

            rospy.loginfo(f"{rospy.get_name()}: GoToTarget - Route: {route}")

            while not rospy.is_shutdown() and not self.stop_event.is_set():
                # Get the top element of the route
                target = route.pop(0)
                rospy.loginfo(
                    f"{rospy.get_name()}: GoToTarget - Going to waypoint %s", target
                )
                # Send target waypoint to the UAV and check if we arrived. If we
                # arrived, send the next waypoint
                self.outer.send_waypoint_uav(target)

                decimator = 0
                while (
                    not self.outer.arrived_at_waypoint(target)
                    and not rospy.is_shutdown()
                    and not self.stop_event.is_set()
                ):
                    # Wait to arrive at the waypoint and publish images at 0.5
                    # hz
                    if decimator % 20 == 0:
                        # Visualize the waypoint sent
                        img = self.outer.planner.display_points(
                            get_image=True, routes=True, plan=True, noFly=True
                        )
                        target_pos = self.outer.planner.mission.waypoints[target]
                        target_px = self.outer.planner.scale_points(
                            target_pos[0], target_pos[1]
                        )
                        robot_target_px = self.outer.planner.scale_points(
                            robot_target.x, robot_target.y
                        )
                        uav_pos = self.outer.uav_pose.pose.position
                        uav_pos_px = self.outer.planner.scale_points(
                            uav_pos.x, uav_pos.y
                        )
                        img = cv2.circle(img, tuple(target_px), 10, (0, 0, 255), 2)
                        img = cv2.circle(
                            img, tuple(robot_target_px), 10, (0, 255, 0), 2
                        )
                        img = cv2.circle(img, tuple(uav_pos_px), 8, (0, 150, 255), -1)
                        self.outer.vis_pub.publish(cv_to_ros(img))
                        decimator = 0
                    decimator += 1
                    rate.sleep()

                # Check if we made it to the end
                if len(route) == 0 and self.outer.arrived_at_waypoint(target):
                    self.outer.set_mode(self.outer.Mode.go_to_target_end)
                    rospy.loginfo(f"{rospy.get_name()}: GoToTarget: reached goal")
                    return


if __name__ == "__main__":
    Navigator()
    rospy.spin()
