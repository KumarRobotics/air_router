#!/usr/bin/env python3
import rospy
from air_router.msg import Goal
import sys
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import os
import importlib
import pdb
import numpy as np

count = 0


def status_callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": Navigator Status Callback: %s", data.data)


def create_goal_msg(action, coord=None):
    if not hasattr(create_goal_msg, "count"):
        create_goal_msg.count = 0
    assert action in ['explore', 'go to robot']
    goal = Goal()
    goal.action = action
    goal.goal.header.seq = create_goal_msg.count
    create_goal_msg.count += 1
    goal.goal.header.stamp = rospy.Time.now()
    if goal == 'go to robot':
        assert coord is not None and len(coord) == 3
        goal.goal.point.x = coord[0]
        goal.goal.point.y = coord[1]
        goal.goal.point.z = coord[2]
    return goal

def create_pose_msg(coord):
    if not hasattr(create_pose_msg, "count"):
        create_pose_msg.count = 0
    pose = PoseStamped()
    pose.header.seq = create_pose_msg.count
    create_pose_msg.count += 1
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = coord[0]
    pose.pose.position.y = coord[1]
    pose.pose.position.z = 60
    return pose


if __name__ == "__main__":
    rospy.init_node('test_navigator', anonymous=False)
    # Test the navigator, by injecting messages mimicking the aerial robot
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--map_name', type=str, required=True)
    args = ap.parse_args()

    # Get map config
    import rospkg
    rospack = rospkg.RosPack()
    semantics_path = rospack.get_path('semantics_manager')
    map_path = os.path.join(semantics_path, "maps", args.map_name, "map_config.yaml")

    # Import mission and get waypoints
    air_router_path = os.path.join(rospack.get_path('air_router'), "scripts")
    sys.path.append(air_router_path)
    import route_planner
    p = route_planner.Path_planner(map_path)
    wp = p.mission.waypoints

    # Create publisher for /unity_ros/quadrotor/Truestate/pose
    pose_pub = rospy.Publisher("/unity_ros/quadrotor/TrueState/pose",
                               PoseStamped, queue_size=10)

    # Create publisher for /air_router/goal
    goal_pub = rospy.Publisher('/air_router/goal',
                                 Goal, queue_size=10)

    # Subscribe to the navigator status
    rospy.Subscriber('/air_router/navigator/status', String, status_callback)

    rospy.loginfo("Starting test navigator")

    # Send initial explore command
    curr_wp_expl = min(wp.keys())
    rospy.sleep(1)
    pose_pub.publish(create_pose_msg(wp[curr_wp_expl]))
    print("Publishing explore")
    goal_pub.publish(create_goal_msg("explore"))
    for i in range(10):
        print("Publishing waypoint %d" % curr_wp_expl)
        pose_pub.publish(create_pose_msg(wp[curr_wp_expl]))
        curr_wp_expl += 1
        rospy.sleep(.2)
    print("Publishing Go To Robot")
    goal_pub.publish(create_goal_msg("go to robot", [0, 0]))
    rospy.sleep(.2)
    print("Publishing waypoint 10")
    pose_pub.publish(create_pose_msg(wp[10]))
    rospy.sleep(.2)
    print("Publishing waypoint 6")
    pose_pub.publish(create_pose_msg(wp[6]))
    rospy.sleep(.2)
    print("Publishing explore")
    goal_pub.publish(create_goal_msg("explore"))
    rospy.sleep(.2)
    print("Publishing waypoint 12")
    pose_pub.publish(create_pose_msg(wp[12]))
    rospy.sleep(.2)
    for i in range(20):
        try:
            pose_pub.publish(create_pose_msg(wp[curr_wp_expl]))
            print("Publishing waypoint %d" % curr_wp_expl)
        except:
            break
        curr_wp_expl += 1
        rospy.sleep(.2)
    print("Publishing Go To Robot")
    goal_pub.publish(create_goal_msg("go to robot", [0, 0]))
    rospy.sleep(.2)
    print("Publishing Go To Robot")
    goal_pub.publish(create_goal_msg("go to robot", [100, 0]))
    rospy.sleep(.2)
    rospy.sleep(5)


