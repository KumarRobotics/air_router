#!/usr/bin/env python3

import os
import pdb
import random
import sys
import yaml

import numpy as np
from router_interfaces.msg import Goal
from colorama import Back, Fore, Style
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, String
from builtin_interfaces.msg import Time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from ament_index_python.packages import get_package_share_directory

from air_router import route_planner


DEFAULT_WORLD_PATH = "/home/jonathan/temp/config.yaml"


class Test(Node):
    def __init__(self):
        super().__init__('test_goal_finder')
    
    def run_test(self):
        self.declare_parameter('map_name', "")
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.declare_parameter('max_edge_length', 100)
        max_edge_length = self.get_parameter('max_edge_length').get_parameter_value().integer_value

        # Get map config

        # rospack = rospkg.RosPack()
        # semantics_path = rospack.get_path("semantics_manager")
        # semantics_path = get_package_share_directory('semantics_manager')
        # map_path = os.path.join(semantics_path, "maps", map_name, "map_config.yaml")

        # # Import mission and get waypoints
        # air_router_path = os.path.join(get_package_share_directory('air_router'), "scripts")
        # sys.path.append(air_router_path)
        
        self.declare_parameter('world_config_path', DEFAULT_WORLD_PATH)
        world_config_path = self.get_parameter('world_config_path').get_parameter_value().string_value

        with open(world_config_path, "r") as f:
            world_config = yaml.safe_load(f)
        path = os.path.dirname(world_config_path)
        my_map = os.path.join(path, world_config["map"])

        p = route_planner.Path_planner(my_map, max_edge_length)
        wp = p.mission.waypoints

        # Create publisher for /unity_ros/quadrotor/Truestate/pose
        pose_pub = self.create_publisher(PoseStamped, 'unity_ros/quadrotor/TrueState/pose', 1)
        goal_pub = self.create_publisher(PointStamped, 'air_router/goal_finder/goal', 1)

        print_test_message("Starting goal finder integration")

        # Send initial pose of the first waypoint
        curr_wp_init = min(wp.keys())
        print_test_message(f"Publishing first pose")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.publish_waypoint(curr_wp_init)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        # Go to the origin
        goal_pub.publish(self.create_point_msg(0, 0, 0))
        self.publish_waypoint(22)
        self.publish_waypoint(27)
        self.publish_waypoint(40)

        # The map has extreme coords x(-160, 120), y(-120, 50)
        # Go to the first extreme
        goal_pub.publish(self.create_point_msg(-160, -120, 0))
        self.publish_waypoint(28)
        self.publish_waypoint(21)
        self.publish_waypoint(2)

        # Go to the second extreme
        goal_pub.publish(self.create_point_msg(120, -120, 0))
        self.publish_waypoint(5)
        self.publish_waypoint(8)
        self.publish_waypoint(12)

        # Go to the third
        goal_pub.publish(self.create_point_msg(-160, 50, 0))
        self.publish_waypoint(16)
        self.publish_waypoint(29)
        # do not continue 43, 46

        # Preempt current search with forth
        goal_pub.publish(self.create_point_msg(120, 50, 0))
        self.publish_waypoint(38)
        self.publish_waypoint(56)

        # Back to origin
        goal_pub.publish(self.create_point_msg(0, 0, 0))

    
    def create_point_msg(self, x, y, z):
        point = PointStamped()
        point.header.stamp = Clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = z
        return point

    def create_pose_msg(self, coord):
        pose = PoseStamped()
        pose.header.stamp = Clock().now().to_msg()
        pose.pose.position.x = coord[0] + random.randint(0, 1)
        pose.pose.position.y = coord[1] + random.randint(0, 1)
        pose.pose.position.z = 60
        return pose

    def publish_waypoint(self, w):
        print_test_message(f"Publishing Titan pose {w}")
        pose_pub.publish(self.create_pose_msg(wp[w]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
    
    def print_test_message(self, msg):
        self.get_logger().info(
            f"{Fore.BLUE}{Style.BRIGHT}{self.get_name()}: " + f"{msg}{Style.RESET_ALL}"
        )


def main(args=None):
    rclpy.init(args=args)

    test_node = Test()

    print(f"Running goal_finder test")
    test_node.run_test()

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()