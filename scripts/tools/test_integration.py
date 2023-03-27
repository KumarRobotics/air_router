#!/usr/bin/env python3

from air_router.msg import Goal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Empty
import numpy as np
import os
import pdb
import rospy
import sys
import random

count = 0

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

def create_robot_pose(robot, coord):
    if not hasattr(create_robot_pose, "count"):
        create_robot_pose.count = 0
    pose = PoseWithCovarianceStamped()
    pose.header.seq = create_robot_pose.count
    create_robot_pose.count += 1
    pose.header.stamp = rospy.Time.now()
    # Add a random offset to the robot pose between 0 and 10
    pose.pose.pose.position.x = coord[0] + random.randint(0, 10)
    pose.pose.pose.position.y = coord[1] + random.randint(0, 10)
    return pose

if __name__ == "__main__":
    rospy.init_node('test_integration', anonymous=False)
    # get map_name from the params
    map_name = rospy.get_param("~map_name")

    # Get map config
    import rospkg
    rospack = rospkg.RosPack()
    semantics_path = rospack.get_path('semantics_manager')
    map_path = os.path.join(semantics_path, "maps", map_name, "map_config.yaml")

    # Import mission and get waypoints
    air_router_path = os.path.join(rospack.get_path('air_router'), "scripts")
    sys.path.append(air_router_path)
    import route_planner
    p = route_planner.Path_planner(map_path)
    wp = p.mission.waypoints

    # Create publisher for /unity_ros/quadrotor/Truestate/pose
    pose_pub = rospy.Publisher("/unity_ros/quadrotor/TrueState/pose",
                               PoseStamped, queue_size=1)

    # Create a publisher for callisto and io poses
    callisto_pose_pub = rospy.Publisher("/callisto/top_down_render/pose_est",
                                        PoseWithCovarianceStamped, queue_size=10)

    callisto_ddb_sync = rospy.Publisher("/ddb/sync_complete/callisto",
                                        Empty, queue_size=10)

    # Create a publisher for callisto and io poses
    io_pose_pub = rospy.Publisher("/io/top_down_render/pose_est",
                                  PoseWithCovarianceStamped, queue_size=10)
    io_ddb_sync = rospy.Publisher("/ddb/sync_complete/io",
                                  Empty, queue_size=10)

    def publish_waypoint(w):
        rospy.loginfo(f"{rospy.get_name()}: Publishing waypoint {w}")
        pose_pub.publish(create_pose_msg(wp[w]))
        rospy.sleep(.2)

    rospy.loginfo("Starting test integration")

    # Send initial explore command
    curr_wp_expl = min(wp.keys())
    rospy.loginfo(f"{rospy.get_name()}: Publishing robot pose")
    rospy.sleep(1)
    pose_pub.publish(create_pose_msg(wp[curr_wp_expl]))
    rospy.sleep(1)

    for i in range(10):
        publish_waypoint(curr_wp_expl)
        curr_wp_expl += 1

    # Test the timeout feature of search when there are no robots to search
    rospy.sleep(10)
    publish_waypoint(curr_wp_expl)

    # Publish a robot pose, now the search should trigger
    rospy.loginfo(f"{rospy.get_name()}: Publishing callisto pose")
    callisto_pose_pub.publish(create_robot_pose("callisto", wp[2]))
    rospy.sleep(10)
    publish_waypoint(6)
    publish_waypoint(2)
    # Timeout the search
    rospy.sleep(10)

    # Finding the robot in a non-search state should not work
    rospy.loginfo(f"{rospy.get_name()}: Publishing callisto pose")
    callisto_ddb_sync.publish()

    # Resume exploration
    publish_waypoint(6)
    publish_waypoint(13)

    for i in range(10):
        publish_waypoint(curr_wp_expl)
        curr_wp_expl += 1
    rospy.sleep(10)

    # Finding the robot in a search should trigger an exploration
    rospy.loginfo(f"{rospy.get_name()}: Publishing callisto pose")
    callisto_pose_pub.publish(create_robot_pose("callisto", wp[5]))
    callisto_ddb_sync.publish()
    rospy.sleep(1)
    publish_waypoint(4)
    publish_waypoint(24)
    publish_waypoint(22)
    for i in range(8):
        publish_waypoint(curr_wp_expl)
        curr_wp_expl += 1

    # Publish Io pose
    io_pose_pub.publish(create_robot_pose("io", wp[10]))
    io_ddb_sync.publish()
    rospy.sleep(11)
    publish_waypoint(23)
    publish_waypoint(19)
    publish_waypoint(13)
    publish_waypoint(10)







