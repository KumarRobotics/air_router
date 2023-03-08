#!/usr/bin/env python3

from air_router.msg import Goal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import os
import pdb
import rospy
import sys

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
    pose.pose.position.x = coord[0]
    pose.pose.position.y = coord[1]
    pose.pose.position.z = 60
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
                               PoseStamped, queue_size=10)

    rospy.loginfo("Starting test integration")

    # Send initial explore command
    curr_wp_expl = min(wp.keys())
    rospy.sleep(1)
    pose_pub.publish(create_pose_msg(wp[curr_wp_expl]))
    for i in range(10):
        print("Publishing waypoint %d" % curr_wp_expl)
        pose_pub.publish(create_pose_msg(wp[curr_wp_expl]))
        curr_wp_expl += 1
        rospy.sleep(.2)
    rospy.sleep(10)
    pose_pub.publish(create_pose_msg(wp[curr_wp_expl]))
    rospy.spin()
    print("Publishing waypoint 10")
    pose_pub.publish(create_pose_msg(wp[10]))
    rospy.sleep(.2)
    print("Publishing waypoint 6")
    pose_pub.publish(create_pose_msg(wp[6]))
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
    rospy.sleep(5)




