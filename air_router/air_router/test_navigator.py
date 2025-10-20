#!/usr/bin/env python3
from router_interfaces.msg import Goal
import sys
import random
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import os
import importlib
import pdb
import numpy as np
import argparse

from air_router import route_planner

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock



class TestNav(Node):
    def __init__(self):
        super().__init__('test_navigator')

    def run_test(self):
        # get map_name from the params
        self.declare_parameter('max_edge_length', 100)
        max_edge_length = self.get_parameter('max_edge_length').get_parameter_value().integer_value

        # Import mission and get waypoints
        self.declare_parameter('config_path', value="PARAMETER NOT SET")
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not os.path.exists(config_path):
            self.get_logger().error(f"Map config file does not exist")
            rclpy.shutdown()
            return

        p = route_planner.Path_planner(config_path, max_edge_length)
        wp = p.mission.waypoints

        # Create publisher for /unity_ros/quadrotor/Truestate/pose
        pose_pub = self.create_publisher(PoseStamped, 'unity_ros/quadrotor/TrueState/pose', 10)

        # Create publisher for /air_router/goal
        goal_pub = self.create_publisher(Goal, 'air_router/goal', 10)

        # Subscribe to the navigator status
        self.create_subscription(String, 'air_router/navigator/status', self.status_callback, 1)

        self.get_logger().info("Starting test navigator")

        # Send initial explore command
        curr_wp_expl = min(wp.keys())
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        pose_pub.publish(self.create_pose_msg(wp[curr_wp_expl]))
        print("Publishing explore")
        goal_pub.publish(self.create_goal_msg("explore"))
        for i in range(10):
            print("Publishing waypoint %d" % curr_wp_expl)
            pose_pub.publish(self.create_pose_msg(wp[curr_wp_expl]))
            curr_wp_expl += 1
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        print("Publishing Go To Robot")
        goal_pub.publish(self.create_goal_msg("go to robot", [0, 0]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        print("Publishing waypoint 10")
        pose_pub.publish(self.create_pose_msg(wp[10]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        print("Publishing waypoint 6")
        pose_pub.publish(self.create_pose_msg(wp[6]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        print("Publishing explore")
        goal_pub.publish(self.create_goal_msg("explore"))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        print("Publishing waypoint 12")
        pose_pub.publish(self.create_pose_msg(wp[12]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        for i in range(20):
            try:
                pose_pub.publish(self.create_pose_msg(wp[curr_wp_expl]))
                print("Publishing waypoint %d" % curr_wp_expl)
            except:
                break
            curr_wp_expl += 1
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        print("Publishing Go To Robot")
        goal_pub.publish(self.create_goal_msg("go to robot", [0, 0]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        print("Publishing Go To Robot")
        goal_pub.publish(self.create_goal_msg("go to robot", [100, 0]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=5.2))


    def status_callback(self, data):
        self.get_logger().info(self.get_name() + ": Navigator Status Callback: %s", data.data)


    def create_goal_msg(self, action, coord=None):
        assert action in ['explore', 'go to robot']
        goal = Goal()
        goal.action = action
        goal.goal.header.stamp = Clock().now().to_msg()
        if goal == 'go to robot':
            assert coord is not None and len(coord) == 3
            goal.goal.point.x = coord[0]
            goal.goal.point.y = coord[1]
            goal.goal.point.z = coord[2]
        return goal


    def create_pose_msg(self, coord):
        pose = PoseStamped()
        pose.header.stamp = Clock().now().to_msg()
        pose.pose.position.x = coord[0]
        pose.pose.position.y = coord[1]
        pose.pose.position.z = 60.0
        return pose



def main(args=None):
    rclpy.init(args=args)

    test_node = TestNav()

    print(f"Running navigator test")
    test_node.run_test()

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()