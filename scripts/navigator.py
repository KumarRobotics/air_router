#!/usr/bin/env python3
from os import wait
import rospy
import route_planner
import os
import yaml
from std_msgs.msg import String
import threading
from air_router.msg import Goal
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from sensor_msgs.msg import NavSatFix
import pdb
import rospkg
import numpy as np
import utm
from mavros_msgs.srv import SetMode, WaypointSetCurrent

"""Navigator:
This node is responsible for navigating the UAV along the waypoints. It
receives a working mode from the state machine("go to robot", "explore") and
creates a plan for performing the action using the router.
"""

# Default acceptance radius for the UAV in meters
DEFAULT_ACCEPTANCE_RADIUS = 2


class Navigator:
    def __init__(self):
        rospy.init_node('navigator', anonymous=False)
        rospy.loginfo(f"{rospy.get_name()}: Started")

        # Get the acceptance radius for the UAV, which should be an integer
        # between 1 and 20
        self.acceptance_radius = rospy.get_param("~acceptance_radius",
                                                 DEFAULT_ACCEPTANCE_RADIUS)
        if not isinstance(self.acceptance_radius, int) or \
                self.acceptance_radius < 1 or self.acceptance_radius > 20:
            rospy.logerr(f"{rospy.get_name()}: \
                    Acceptance radius should be an integer between 1 and 20")
            rospy.signal_shutdown("Acceptance radius error")
            return

        # Are we in simulator mode?
        self.sim = rospy.get_param("~sim", True)

        # Get the path to the map file
        pkg = rospkg.RosPack()
        path = pkg.get_path('semantics_manager')
        if not rospy.has_param("~map_name"):
            rospy.logfatal(f"{rospy.get_name()}: map_name is not set")
            rospy.signal_shutdown("Map name is not set")
            return
        self.map_name = rospy.get_param("~map_name")
        assert self.map_name is not None, "Map name is not set"
        assert isinstance(self.map_name, str), "Map name should be a string"
        # Does the map path exists?
        if not os.path.exists(os.path.join(path, "maps", self.map_name)):
            rospy.logfatal(f"{rospy.get_name()}: Map {self.map_name} does not exist")
            rospy.signal_shutdown("Map does not exist")
            return
        # Does the map config file exist?
        config_file = os.path.join(path, "maps",
                                   self.map_name, "map_config.yaml")
        if not os.path.exists(config_file):
            rospy.logfatal(f"{rospy.get_name()}: Map config file does not exist")
            rospy.signal_shutdown("Map config file does not exist")
            return
        rospy.loginfo(f"{rospy.get_name()}: Map: {self.map_name}")
        rospy.loginfo(f"{rospy.get_name()}: Sim: {self.sim}")
        rospy.loginfo(f"{rospy.get_name()}: AR: {self.acceptance_radius}")

        # Create a path planner object
        self.planner = route_planner.Path_planner(self.map_name)

        # Initially, the navigator is in the "idle" mode. We will wait for an
        # order from the state machine
        self.mode = None
        self.mode_pub = rospy.Publisher("/air_router/navigator/status",
                                        String, queue_size=10)
        self.waypoint_list = list(self.planner.mission.waypoints.keys())
        self.explore_target_waypt = self.waypoint_list.copy()
        self.uav_pose = None

        # Target goal for the UAV in GoToTarget mode
        self.robot_target = None

        # Create threads for the different modes
        self.stop_exploration = threading.Event()
        self.stop_go_to_target = threading.Event()

        # Create subscribers for the state machine topics: goal and coordinates
        rospy.Subscriber("/air_router/goal", Goal, self.goal_callback)

        # Create a subscriber for the UAV position. This is for the simulator.
        # For the real world, we will use the GPS input here
        if self.sim:
            rospy.Subscriber("/unity_ros/quadrotor/TrueState/pose",
                             PoseStamped, self.pose_callback)
        else:
            rospy.Subscriber("/mavros/global_position/global",
                             NavSatFix, self.gps_callback)

        # Publish the goal for the UAV. For simulation, we will just publish a
        # goal, for the real world, we will use the mavros interface
        if self.sim:
            self.uav_goal = rospy.Publisher("/quadrotor/goal",
                                            PointStamped, queue_size=10)
        else:
            # Service proxy for /mavros/misssion/set_current
            rospy.wait_for_service("/mavros/mission/set_current")
            self.set_cur_wp = rospy.ServiceProxy("/mavros/mission/set_current",
                                                 WaypointSetCurrent)

        # Counters for ROS msgs
        self.uav_goal_seq = 0
        self.uav_pose_seq = 0

    def set_mode(self, mode):
        self.mode = mode
        self.mode_pub.publish(self.mode)

    def goal_callback(self, data):
        if self.mode is None and data.action == "explore":
            self.set_mode("explore")
            # initial explore
            self.explore_thread = self.ExplorationThread(self,
                                                         self.stop_exploration)
            self.explore_thread.daemon = True
            self.explore_thread.start()

        elif self.mode == "explore" and data.action == "go to robot":
            # Signal the exploration thread to stop
            self.stop_exploration.set()
            self.explore_thread.join()
            # Go find the robot
            self.set_mode("go to robot")
            self.robot_target = data.goal.point
            self.goto_robot_thread = self.GoToTargetThread(self,
                                                           self.stop_go_to_target)
            self.goto_robot_thread.daemon = True
            self.goto_robot_thread.start()
        elif self.mode == "go to robot" and data.action == "explore":
            # Signal the go to robot thread to stop
            self.stop_go_to_target.set()
            self.goto_robot_thread.join()
            self.set_mode("returning")
            # Go to the last exploration position
            p = self.planner.mission.waypoints[self.explore_target_waypt[0]]
            self.robot_target = Point(p[0], p[1], 60)
            self.goto_robot_thread = self.GoToTargetThread(self, self.stop_go_to_target)
            self.goto_robot_thread.daemon = True
            self.goto_robot_thread.start()
            self.goto_robot_thread.join()
            # Resume exploration
            self.set_mode("explore")
            self.explore_thread = self.ExplorationThread(self,
                                                         self.stop_exploration)
            self.explore_thread.daemon = True
            self.explore_thread.start()
        else:
            # unknown transition. Die
            rospy.logerr("Unknown transition from %s to %s",
                         self.mode, data.action)
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
            target.point.x = waypoint[0]
            target.point.y = waypoint[1]
            target.point.z = 60
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
            curr = np.array([self.uav_pose.pose.position.x,
                             self.uav_pose.pose.position.y])
            # print(f"Current position: {target}, target: {wp}")
            if np.linalg.norm(curr - wp) < self.acceptance_radius:
                return True
        return False

    class ExplorationThread(threading.Thread):
        def __init__(self, outer, stop_event):
            super().__init__()
            self.stop_event = stop_event
            self.outer = outer

        def run(self):
            self.stop_event.clear()
            rate = rospy.Rate(10)
            rospy.loginfo(f"{rospy.get_name()}: Exploration - Start")
            while not rospy.is_shutdown() and not self.stop_event.is_set():
                # Get the top element on the list as target waypoint
                target = self.outer.explore_target_waypt[0]
                # Send target waypoint to the UAV and check if we arrived. If we
                # arrived, send the next waypoint
                self.outer.send_waypoint_uav(target)
                rospy.loginfo(f"{rospy.get_name()}: Exploration - Going to waypoint %s", target)
                while (not self.outer.arrived_at_waypoint(target) and
                       not rospy.is_shutdown() and
                       not self.stop_event.is_set()):
                    # Wait to arrive at the waypoint
                    rate.sleep()
                if self.outer.arrived_at_waypoint(target):
                    self.outer.explore_target_waypt = self.outer.explore_target_waypt[1:] + [target]
            rospy.loginfo(f"{rospy.get_name()}: Exploration - End")

    class GoToTargetThread(threading.Thread):
        def __init__(self, outer, stop_event):
            super().__init__()
            self.stop_event = stop_event
            self.outer = outer

        def run(self):
            self.stop_event.clear()
            rate = rospy.Rate(10)

            rospy.loginfo(f"{rospy.get_name()}: GoToTarget - Start")

            # Wait until we have a valid robot pose
            while (not rospy.is_shutdown() and
                   not self.stop_event.is_set() and
                   self.outer.uav_pose is None):
                rate.sleep()

            # Get the current position of the robot and target
            pos = self.outer.uav_pose.pose.position
            robot_target = self.outer.robot_target

            # Get the trajectory from the current position to the target
            route = self.outer.planner.planRoute([pos.x, pos.y],
                                                 [robot_target.x,
                                                  robot_target.y])
            if route is None:
                rospy.logerr(f"{rospy.get_name()}: GoToTarget - Could not find route.")
                return

            if len(route) == 0:
                rospy.loginfo(f"{rospy.get_name()}: GoToTarget - Already at the target.")

            # Check if we are already at the first waypoint
            if len(route) > 0 and self.outer.arrived_at_waypoint(route[0]):
                route = route[1:]

            rospy.loginfo(f"{rospy.get_name()}: GoToTarget - Route: {route}")

            while not rospy.is_shutdown() and not self.stop_event.is_set():
                # Get the top element of the route
                target = route.pop(0)
                rospy.loginfo(f"{rospy.get_name()}: GoToTarget - Going to waypoint %s",
                              target)
                # Send target waypoint to the UAV and check if we arrived. If we
                # arrived, send the next waypoint
                self.outer.send_waypoint_uav(target)
                while (not self.outer.arrived_at_waypoint(target) and
                       not rospy.is_shutdown() and
                       not self.stop_event.is_set()):
                    # Wait to arrive at the waypoint
                    rate.sleep()
                # Check if we made it to the end
                if len(route) == 0:
                    rospy.loginfo(f"{rospy.get_name()}: GoToTarget: reached goal")
                    return


if __name__ == "__main__":
    Navigator()
    rospy.spin()
