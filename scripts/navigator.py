#!/usr/bin/env python3
from os import wait
import rospy
import route_planner
import argparse
import os
import yaml
from std_msgs.msg import String
import threading
from air_router.msg import Goal
from geometry_msgs.msg import PointStamped, PoseStamped, Point
import pdb
import rospkg
import numpy as np

# Acceptance radius for the UAV in meters
ACCEPTANCE_RADIUS=2

"""Navigator class
This class is responsible for navigating the UAV along the waypoints. It
receives a working mode from the state machine("go to robot", "explore") and
creates a plan for performing the action using the router.
"""
class Navigator:
    def __init__(self, map_name):
        rospy.init_node('navigator', anonymous=False)
        rospy.loginfo(f"{rospy.get_name()}: Started")

        self.map_name = map_name

        # Get the path to the mission file
        pkg = rospkg.RosPack()
        path = pkg.get_path('semantics_manager')
        map_yaml_path = os.path.join(path, "maps", args.map_name, "map_config.yaml")
        with open(map_yaml_path, "r") as f:
            map_yaml = yaml.load(f, Loader=yaml.FullLoader)
        mission_file = os.path.join(path, "maps", args.map_name, map_yaml["quad_plan"])

        # Create a path planner and mission objects
        self.mission = route_planner.Mission(mission_file, map_yaml["quad_plan_format"])
        self.planner = route_planner.Path_planner(self.mission, self.map_name)

        # We will wait for a mode from the state machine
        self.mode = None
        self.waypoint_list = list(self.mission.waypoints.keys())
        self.explore_target_waypt = self.waypoint_list.copy()
        self.uav_pose = None

        self.robot_target = None

        # Create threads for the different modes
        self.stop_exploration = threading.Event()
        self.stop_goto_robot = threading.Event()

        # Create subscribers for the state machine topics: goal and coordinates
        rospy.Subscriber("/air_router/goal", Goal, self.goal_callback)

        # Create a subscriber for the UAV position. This is for the simulator.
        # For the real world, we will use the GPS input here
        rospy.Subscriber("/unity_ros/quadrotor/TrueState/pose",
                         PoseStamped, self.pose_callback)

        # Publish the goal for the UAV. For simulation, we will just publish a
        # goal, for the real world, we will use the mavros interface
        self.uav_goal = rospy.Publisher("/quadrotor/goal",
                                        PointStamped, queue_size=10)
        self.uav_goal_seq = 0


    def goal_callback(self, data):
        if self.mode is None and data.action == "explore":
            self.mode = "explore"
            # initial explore
            self.explore_thread = self.ExplorationThread(self, self.stop_exploration)
            self.explore_thread.daemon = True
            self.explore_thread.start()

        elif self.mode == "explore" and data.action == "go to robot":
            # Signal the exploration thread to stop
            self.stop_exploration.set()
            self.explore_thread.join()
            # Loiter at the current position
            self.send_loiter_uav()
            # Go find the robot
            self.mode = "go to robot"
            self.robot_target = data.goal.point
            self.goto_robot_thread = self.GoToTargetThread(self, self.stop_goto_robot)
            self.goto_robot_thread.daemon = True
            self.goto_robot_thread.start()
        elif self.mode == "go to robot" and data.action == "explore":
            # Signal the go to robot thread to stop
            self.stop_goto_robot.set()
            self.goto_robot_thread.join()
            # Loiter at the current position
            self.send_loiter_uav()
            # Go to the last exploration position
            p = self.mission.waypoints[self.explore_target_waypt[0]]
            self.robot_target = Point(p[0], p[1], 60)
            self.goto_robot_thread = self.GoToTargetThread(self, self.stop_goto_robot)
            self.goto_robot_thread.daemon = True
            self.goto_robot_thread.start()
            self.goto_robot_thread.join()
            # Resume exploration
            self.mode = "explore"
            self.explore_thread = self.ExplorationThread(self, self.stop_exploration)
            self.explore_thread.daemon = True
            self.explore_thread.start()
        else:
            # unknown transition. Die
            rospy.logerr("Unknown transition from %s to %s", self.mode, data.action)
            rospy.signal_shutdown("Shutting down navigator")

    def pose_callback(self, data):
        self.uav_pose = data

    def send_waypoint_uav(self, target_wpt):
        # For simulation purposes, we will publish the target waypoint
        target = PointStamped()
        target.header.seq = self.uav_goal_seq
        self.uav_goal_seq += 1
        target.header.stamp = rospy.Time.now()
        waypoint = self.mission.waypoints[target_wpt]
        target.point.x = waypoint[0]
        target.point.y = waypoint[1]
        target.point.z = 60
        self.uav_goal.publish(target)
        # For the quad, we are calling the mavros interface to do it
        # TODO

    def send_loiter_uav(self):
        # For simulation purposes, we will publish the current position of the
        # robot
        if self.uav_pose is None:
            return
        target = PointStamped()
        target.header.seq = self.uav_goal_seq
        self.uav_goal_seq += 1
        target.header.stamp = rospy.Time.now()
        target.point.x = self.uav_pose.pose.position.x
        target.point.y = self.uav_pose.pose.position.y
        target.point.z = self.uav_pose.pose.position.z
        self.uav_goal.publish(target)
        # TODO: for the real quad, we should send the loiter cmd

    def arrived_at_waypoint(self, waypoint):
        if self.uav_pose is not None:
            wp = self.mission.waypoints[waypoint]
            target = np.array([self.uav_pose.pose.position.x,
                               self.uav_pose.pose.position.y])
            # print(f"Current position: {target}, target: {wp}")
            if np.linalg.norm(target - wp) < ACCEPTANCE_RADIUS:
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
            rospy.loginfo(f"{rospy.get_name()}: Starting exploration")
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
            rospy.loginfo(f"{rospy.get_name()}: End exploration")

    class GoToTargetThread(threading.Thread):
        def __init__(self, outer, stop_event):
            super().__init__()
            self.stop_event = stop_event
            self.outer = outer

        def run(self):
            self.stop_event.clear()
            rate = rospy.Rate(10)

            rospy.loginfo(f"{rospy.get_name()}: Starting Go To Robot")

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
                rospy.logerr(f"{rospy.get_name()}: Could not find a route to the robot")

            # Check if we are already at the first waypoint
            if len(route) > 0 and self.outer.arrived_at_waypoint(route[0]):
                route = route[1:]

            rospy.loginfo(f"{rospy.get_name()}: GoToTarget - Route: {route}")

            while not rospy.is_shutdown() and not self.stop_event.is_set():
                # Get the top element of the route
                target = route.pop(0)
                rospy.loginfo(f"{rospy.get_name()}: GoToTarget - Going to waypoint %s", target)
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
                    rospy.loginfo(f"{rospy.get_name()}: GoToTarget: we reached the target")
                    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_name", help="Name of the map to use",
                        required=True)
    args = parser.parse_args()
    Navigator(args.map_name)
    rospy.spin()
