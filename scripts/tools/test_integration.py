#!/usr/bin/env python3

from air_router.msg import Goal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Empty, Time
import numpy as np
import os
import pdb
import rospy
import sys
import random
from colorama import Fore, Back, Style

count = 0


def print_test_message(msg):
    rospy.loginfo(f"{Fore.BLUE}{Style.BRIGHT}{rospy.get_name()}: " +
                  f"{msg}{Style.RESET_ALL}")


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


def create_robot_pose(coord):
    if not hasattr(create_robot_pose, "count"):
        create_robot_pose.count = 0
    pose = PoseWithCovarianceStamped()
    pose.header.seq = create_robot_pose.count
    create_robot_pose.count += 1
    pose.header.stamp = rospy.Time.now()
    # Add a random offset to the robot pose between 0 and 1
    pose.pose.pose.position.x = coord[0] + random.randint(0, 1)
    pose.pose.pose.position.y = coord[1] + random.randint(0, 1)
    return pose

if __name__ == "__main__":
    rospy.init_node('test_integration', anonymous=False)
    # get map_name from the params
    map_name = rospy.get_param("~map_name")
    max_edge_length = rospy.get_param("~max_edge_length")

    # Get map config
    import rospkg
    rospack = rospkg.RosPack()
    semantics_path = rospack.get_path('semantics_manager')
    map_path = os.path.join(semantics_path, "maps",
                            map_name, "map_config.yaml")

    # Import mission and get waypoints
    air_router_path = os.path.join(rospack.get_path('air_router'), "scripts")
    sys.path.append(air_router_path)
    import route_planner
    p = route_planner.Path_planner(map_path, max_edge_length)
    wp = p.mission.waypoints

    # Create publisher for /unity_ros/quadrotor/Truestate/pose
    pose_pub = rospy.Publisher("/unity_ros/quadrotor/TrueState/pose",
                               PoseStamped, queue_size=1)

    # Create a publisher for callisto and io poses
    callisto_pose_pub = rospy.Publisher("/callisto/top_down_render/pose_est",
                                        PoseWithCovarianceStamped, queue_size=10)

    callisto_ddb_sync = rospy.Publisher("/ddb/server_sync_complete/callisto",
                                        Time, queue_size=10)

    # Create a publisher for callisto and io poses
    io_pose_pub = rospy.Publisher("/io/top_down_render/pose_est",
                                  PoseWithCovarianceStamped, queue_size=10)
    io_ddb_sync = rospy.Publisher("/ddb/server_sync_complete/io",
                                  Time, queue_size=10)

    # Create a publisher for the basestation
    basestation_ddb_sync = rospy.Publisher("ddb/server_sync_complete/basestation",
                                           Time, queue_size=10)

    def publish_waypoint(w):
        print_test_message(f"Publishing Titan pose {w}")
        if rospy.is_shutdown():
            sys.exit(0)
        pose_pub.publish(create_pose_msg(wp[w]))
        rospy.sleep(.2)

    print_test_message("Starting test integration")

    # Send initial explore command
    curr_wp_expl = min(wp.keys())
    print_test_message(f"Publishing first pose")
    rospy.sleep(1)
    publish_waypoint(curr_wp_expl)
    rospy.sleep(1)

    for i in range(5):
        if rospy.is_shutdown():
            sys.exit(1)
        publish_waypoint(curr_wp_expl)
        curr_wp_expl += 1

    # Test the timeout feature of search when there are no robots to search
    print_test_message("Should go to search and " +
                       "then go back as there are no targets.")
    rospy.sleep(6)

    # Publish some more robot waypoint
    for i in range(3):
        if rospy.is_shutdown():
            sys.exit(1)
        publish_waypoint(curr_wp_expl)
        curr_wp_expl += 1

    # This should do nothing
    print_test_message("Publishing basestation sync message. " +
                       "should do nothing.")
    basestation_ddb_sync.publish()

    # Publish a robot pose
    print_test_message("Publishing callisto pose")
    callisto_pose_pub.publish(create_robot_pose(wp[2]))
    print_test_message("Time out into search mode")
    rospy.sleep(3)
    print_test_message("Going to callisto pose")
    publish_waypoint(4)
    publish_waypoint(2)
    # Timeout the search
    print_test_message("Do not publish callisto sync complete and time out")
    rospy.sleep(5)
    # Finding the robot in a non-search state should not work
    print_test_message(f"Publishing sync complete, should do nothing")
    callisto_ddb_sync.publish()

    # Resume exploration
    publish_waypoint(6)
    publish_waypoint(7)

    for i in range(3):
        publish_waypoint(curr_wp_expl)
        curr_wp_expl += 1
    print_test_message(f"Timing out into search")
    rospy.sleep(5)

    # Finding the robot in a search should trigger an exploration
    publish_waypoint(11)
    publish_waypoint(7)
    rospy.sleep(1)
    print_test_message(f"Publishing callisto pose. Nothing should " +
                       "happen as we are searching the basestation")
    callisto_pose_pub.publish(create_robot_pose(wp[5]))
    rospy.sleep(1)
    publish_waypoint(6)
    publish_waypoint(2)
    print_test_message("Publish basestation sync complete")
    basestation_ddb_sync.publish()
    rospy.sleep(1)
    print_test_message("Going back to exploration. Publish Io pose.")
    publish_waypoint(6)
    io_pose_pub.publish(create_robot_pose(wp[10]))
    publish_waypoint(7)
    publish_waypoint(14)
    publish_waypoint(13)
    for i in range(10):
        publish_waypoint(curr_wp_expl)
        curr_wp_expl += 1
    rospy.sleep(5)
    # Find Io right away
    io_ddb_sync.publish()
    rospy.sleep(1)
    # Finish exploration mission
    print_test_message("Finish exploration mission")
    publish_waypoint(16)
    publish_waypoint(17)
    publish_waypoint(20)
    publish_waypoint(21)
    publish_waypoint(24)
    publish_waypoint(23)
    for i in range(9):
        publish_waypoint(curr_wp_expl)
        curr_wp_expl += 1
    # Search for callisto, chasing it
    print_test_message("Chase callisto, first target after exploration")
    publish_waypoint(27)
    callisto_pose_pub.publish(create_robot_pose(wp[20]))
    rospy.sleep(.3)
    publish_waypoint(3)
    callisto_pose_pub.publish(create_robot_pose(wp[20]))
    rospy.sleep(.3)
    publish_waypoint(27)
    callisto_pose_pub.publish(create_robot_pose(wp[20]))
    rospy.sleep(.3)
    publish_waypoint(25)
    callisto_pose_pub.publish(create_robot_pose(wp[20]))
    rospy.sleep(.3)
    publish_waypoint(24)
    print_test_message("Timeout")
    rospy.sleep(11)
    print_test_message("Find basestation right away. Publish sync complete.")
    basestation_ddb_sync.publish()
    rospy.sleep(1)
    print_test_message("Searching for io")
    publish_waypoint(21)
    publish_waypoint(20)
    publish_waypoint(17)
    print_test_message("Change io pose")
    io_pose_pub.publish(create_robot_pose(wp[20]))
    rospy.sleep(1)
    print_test_message("Go to the new io position")
    publish_waypoint(7)
    for i in range(2, 13):
        print_test_message("Change io pose")
        io_pose_pub.publish(create_robot_pose(wp[i]))
        rospy.sleep(1)
    print_test_message("Sync complete io. Should do nothing as we already timed out.")
    io_ddb_sync.publish()
    rospy.sleep(1)
    print_test_message("Sync complete callisto")
    callisto_ddb_sync.publish()
    # Finished all tests
    rospy.spin()
