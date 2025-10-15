#!/usr/bin/env python3
from router_interfaces.msg import Goal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Empty
from builtin_interfaces.msg import Time
import numpy as np
import yaml
import os
import pdb
import sys
import random
from colorama import Fore, Back, Style


import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from air_router import route_planner


DEFAULT_WORLD_PATH = "/home/jonathan/temp/pennovation/config.yaml"



class TestNav(Node):
    def __init__(self):
        super().__init__('test_integration')

    def run_test(self):
        # get map_name from the params
        self.declare_parameter('max_edge_length', 100)
        max_edge_length = self.get_parameter('max_edge_length').get_parameter_value().integer_value

        # Import mission and get waypoints
        self.declare_parameter('world_config_path', DEFAULT_WORLD_PATH)
        world_config_path = self.get_parameter('world_config_path').get_parameter_value().string_value

        with open(world_config_path, "r") as f:
            world_config = yaml.safe_load(f)
        path = os.path.dirname(world_config_path)
        my_map = os.path.join(path, world_config["map"])

        # Import mission and get waypoints
        p = route_planner.Path_planner(my_map, max_edge_length)
        wp = p.mission.waypoints

        # Create publisher for /unity_ros/quadrotor/Truestate/pose
        self.pose_pub = self.create_publisher(PoseStamped, 'unity_ros/quadrotor/TrueState/pose', 1)

        # Create a publisher for callisto and io poses
        callisto_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'callisto/top_down_render/pose_est', 10)
        callisto_ddb_sync = self.create_publisher(Time, 'ddb/server_sync_complete/callisto', 10)

        # Create a publisher for callisto and io poses
        io_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'io/top_down_render/pose_est', 10)
        io_ddb_sync = self.create_publisher(Time, 'ddb/server_sync_complete/io', 10)

        # Create a publisher for the basestation
        basestation_ddb_sync = self.create_publisher(Time, 'ddb/server_sync_complete/basestation', 10)

        self.print_test_message("Starting test integration")

        # Send initial explore command
        curr_wp_expl = min(wp.keys())
        self.print_test_message(f"Publishing first pose")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.publish_waypoint(curr_wp_expl)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))

        for i in range(5):
            if not rclpy.ok():
                sys.exit(1)
            self.publish_waypoint(curr_wp_expl)
            curr_wp_expl += 1

        # Test the timeout feature of search when there are no robots to search
        self.print_test_message("Should go to search and " +
                        "then go back as there are no targets.")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=6.0))

        # Publish some more robot waypoint
        for i in range(3):
            if not rclpy.ok():
                sys.exit(1)
            self.publish_waypoint(curr_wp_expl)
            curr_wp_expl += 1

        # This should do nothing
        self.print_test_message("Publishing basestation sync message. " +
                        "should do nothing.")
        basestation_ddb_sync.publish()

        # Publish a robot pose
        self.print_test_message("Publishing callisto pose")
        callisto_pose_pub.publish(self.create_robot_pose(wp[2]))
        self.print_test_message("Time out into search mode")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3.0))
        self.print_test_message("Going to callisto pose")
        self.publish_waypoint(4)
        self.publish_waypoint(2)
        # Timeout the search
        self.print_test_message("Do not publish callisto sync complete and time out")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=5.0))
        # Finding the robot in a non-search state should not work
        self.print_test_message(f"Publishing sync complete, should do nothing")
        callisto_ddb_sync.publish()

        # Resume exploration
        self.publish_waypoint(6)
        self.publish_waypoint(7)

        for i in range(3):
            self.publish_waypoint(curr_wp_expl)
            curr_wp_expl += 1
        self.print_test_message(f"Timing out into search")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=5.0))

        # Finding the robot in a search should trigger an exploration
        self.publish_waypoint(11)
        self.publish_waypoint(7)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.print_test_message(f"Publishing callisto pose. Nothing should " +
                        "happen as we are searching the basestation")
        callisto_pose_pub.publish(self.create_robot_pose(wp[5]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.publish_waypoint(6)
        self.publish_waypoint(2)
        self.print_test_message("Publish basestation sync complete")
        basestation_ddb_sync.publish()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.print_test_message("Going back to exploration. Publish Io pose.")
        self.publish_waypoint(6)
        io_pose_pub.publish(self.create_robot_pose(wp[10]))
        self.publish_waypoint(7)
        self.publish_waypoint(14)
        self.publish_waypoint(13)
        for i in range(10):
            self.publish_waypoint(curr_wp_expl)
            curr_wp_expl += 1
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=5.0))
        # Find Io right away
        io_ddb_sync.publish()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        # Finish exploration mission
        self.print_test_message("Finish exploration mission")
        self.publish_waypoint(16)
        self.publish_waypoint(17)
        self.publish_waypoint(20)
        self.publish_waypoint(21)
        self.publish_waypoint(24)
        self.publish_waypoint(23)
        for i in range(9):
            self.publish_waypoint(curr_wp_expl)
            curr_wp_expl += 1
        # Search for callisto, chasing it
        self.print_test_message("Chase callisto, first target after exploration")
        self.publish_waypoint(27)
        callisto_pose_pub.publish(self.create_robot_pose(wp[20]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))
        self.publish_waypoint(3)
        callisto_pose_pub.publish(self.create_robot_pose(wp[20]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))
        self.publish_waypoint(27)
        callisto_pose_pub.publish(self.create_robot_pose(wp[20]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))
        self.publish_waypoint(25)
        callisto_pose_pub.publish(self.create_robot_pose(wp[20]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))
        self.publish_waypoint(24)
        self.print_test_message("Timeout")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=11))
        self.print_test_message("Find basestation right away. Publish sync complete.")
        basestation_ddb_sync.publish()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.print_test_message("Searching for io")
        self.publish_waypoint(21)
        self.publish_waypoint(20)
        self.publish_waypoint(17)
        self.print_test_message("Change io pose")
        io_pose_pub.publish(self.create_robot_pose(wp[20]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.print_test_message("Go to the new io position")
        self.publish_waypoint(7)
        for i in range(2, 13):
            self.print_test_message("Change io pose")
            io_pose_pub.publish(self.create_robot_pose(wp[i]))
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.print_test_message("Sync complete io. Should do nothing as we already timed out.")
        io_ddb_sync.publish()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        self.print_test_message("Sync complete callisto")
        callisto_ddb_sync.publish()


    def print_test_message(self, msg):
        self.get_logger().info(f"{Fore.BLUE}{Style.BRIGHT}{self.get_name()}: " +
                    f"{msg}{Style.RESET_ALL}")


    def create_pose_msg(self, coord):
        pose = PoseStamped()
        pose.header.stamp = Clock().now().to_msg()
        pose.pose.position.x = coord[0] + random.randint(0, 1)
        pose.pose.position.y = coord[1] + random.randint(0, 1)
        pose.pose.position.z = 60
        return pose


    def create_robot_pose(self, coord):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = Clock().now().to_msg()
        # Add a random offset to the robot pose between 0 and 1
        pose.pose.pose.position.x = coord[0] + random.randint(0, 1)
        pose.pose.pose.position.y = coord[1] + random.randint(0, 1)
        return pose


    def publish_waypoint(self, w):
        self.print_test_message(f"Publishing Titan pose {w}")
        if not rclpy.ok():
            sys.exit(0)
        self.pose_pub.publish(self.create_pose_msg(wp[w]))
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))




def main(args=None):
    rclpy.init(args=args)

    test_node = TestNav()

    print(f"Running navigator test")
    test_node.run_test()

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()