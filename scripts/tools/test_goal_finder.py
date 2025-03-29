#!/usr/bin/env python3

import os
import pdb
import random
import sys

import numpy as np
import rospy
from air_router.msg import Goal
from colorama import Back, Fore, Style
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, String, Time

count = 0


def print_test_message(msg):
    rospy.loginfo(
        f"{Fore.BLUE}{Style.BRIGHT}{rospy.get_name()}: " + f"{msg}{Style.RESET_ALL}"
    )


def goal_cb(msg):
    pass


def create_pose_msg(coord):
    if not hasattr(create_pose_msg, "count"):
        create_pose_msg.count = 0
    pose = PoseStamped()
    pose.header.seq = create_pose_msg.count
    create_pose_msg.count += 1
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = coord[0] + random.randint(0, 1)
    pose.pose.position.y = coord[1] + random.randint(0, 1)
    pose.pose.position.z = 60
    return pose


def create_point_msg(x, y, z):
    if not hasattr(create_point_msg, "count"):
        create_point_msg.count = 0
    point = PointStamped()
    point.header.seq = create_point_msg.count
    create_point_msg.count += 1
    point.header.stamp = rospy.Time.now()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point


if __name__ == "__main__":
    rospy.init_node("test_goal_finder", anonymous=False)
    # get map_name from the params
    map_name = rospy.get_param("~map_name")
    max_edge_length = rospy.get_param("~max_edge_length")

    # Get map config
    import rospkg

    rospack = rospkg.RosPack()
    semantics_path = rospack.get_path("semantics_manager")
    map_path = os.path.join(semantics_path, "maps", map_name, "map_config.yaml")

    # Import mission and get waypoints
    air_router_path = os.path.join(rospack.get_path("air_router"), "scripts")
    sys.path.append(air_router_path)
    import route_planner

    p = route_planner.Path_planner(map_path, max_edge_length)
    wp = p.mission.waypoints

    # Create publisher for /unity_ros/quadrotor/Truestate/pose
    pose_pub = rospy.Publisher(
        "/unity_ros/quadrotor/TrueState/pose", PoseStamped, queue_size=1
    )

    goal_pub = rospy.Publisher(
        "/air_router/goal_finder/goal", PointStamped, queue_size=1
    )

    def publish_waypoint(w):
        print_test_message(f"Publishing Titan pose {w}")
        if rospy.is_shutdown():
            sys.exit(0)
        pose_pub.publish(create_pose_msg(wp[w]))
        rospy.sleep(0.2)

    print_test_message("Starting goal finder integration")

    # Send initial pose of the first waypoint
    curr_wp_init = min(wp.keys())
    print_test_message(f"Publishing first pose")
    rospy.sleep(1)
    publish_waypoint(curr_wp_init)
    rospy.sleep(2)

    # Go to the origin
    goal_pub.publish(create_point_msg(0, 0, 0))
    publish_waypoint(22)
    publish_waypoint(27)
    publish_waypoint(40)

    # The map has extreme coords x(-160, 120), y(-120, 50)
    # Go to the first extreme
    goal_pub.publish(create_point_msg(-160, -120, 0))
    publish_waypoint(28)
    publish_waypoint(21)
    publish_waypoint(2)

    # Go to the second extreme
    goal_pub.publish(create_point_msg(120, -120, 0))
    publish_waypoint(5)
    publish_waypoint(8)
    publish_waypoint(12)

    # Go to the third
    goal_pub.publish(create_point_msg(-160, 50, 0))
    publish_waypoint(16)
    publish_waypoint(29)
    # do not continue 43, 46

    # Preempt current search with forth
    goal_pub.publish(create_point_msg(120, 50, 0))
    publish_waypoint(38)
    publish_waypoint(56)

    # Back to origin
    goal_pub.publish(create_point_msg(0, 0, 0))

    # Finished all tests
    rospy.spin()
