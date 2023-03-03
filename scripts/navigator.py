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
from geometry_msgs.msg import PointStamped, PoseStamped
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
        rospy.init_node('navigator', anonymous=True)

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

        # Create threads for the different modes
        self.explore_thread = threading.Thread(target=self.exploration_thread)
        self.explore_thread.daemon = True
        self.goto_robot_thread = threading.Thread(target=self.goto_robot_thread)
        self.goto_robot_thread.daemon = True

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
            rospy.loginfo("Starting exploration")
            # initial explore
            self.explore_thread.start()

        elif self.mode == "explore" and data.action == "go to robot":
            rospy.loginfo("Going to robot")
            self.mode = "go to robot"
            # transition to go to robot
            pass
        elif self.mode == "go to robot" and data.action == "explore":
            rospy.loginfo("Resuming exploration")
            self.mode = "explore"
            # transition to explore
            pass
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
        # rospy.loginfo(f"Sending waypoint {target_wpt} {waypoint[0]} {waypoint[1]}")
        self.uav_goal.publish(target)

        # For the quad, we are calling the mavros interface to do it
        # TODO

    def arrived_at_waypoint(self, waypoint):
        if self.uav_pose is not None:
            wp = self.mission.waypoints[waypoint]
            target = np.array([self.uav_pose.pose.position.x,
                               self.uav_pose.pose.position.y])
            if np.linalg.norm(target - wp) < ACCEPTANCE_RADIUS:
                return True
        return False

    def exploration_thread(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Get the top element on the list as target waypoint
            target = self.explore_target_waypt[0]
            rospy.loginfo("Going to waypoint %s", target)
            self.explore_target_waypt = self.explore_target_waypt[1:] + [target]
            # Send target waypoint to the UAV and check if we arrived. If we
            # arrived, send the next waypoint
            self.send_waypoint_uav(target)
            while (not self.arrived_at_waypoint(target) and
                   not rospy.is_shutdown()):
                # Wait to arrive at the waypoint
                rate.sleep()

    def goto_robot_thread(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_name", help="Name of the map to use",
                        required=True)
    args = parser.parse_args()
    Navigator(args.map_name)
    rospy.spin()
