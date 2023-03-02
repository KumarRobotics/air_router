#!/usr/bin/env python3
import rospy
import route_planner
from std_msgs.msg import String
import threading
from air_router.msg import Goal
from geometry_msgs.msg import PointStamped
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
    def __init__(self):
        rospy.init_node('navigator', anonymous=True)
        # Create a path planner object within the navigator
        rospack = rospkg.RosPack()
        this_path = rospack.get_path('air_router')
        self.planner = route_planner.Path_planner(this_path + "/sample_data/Pennovation_Ian_2023.plan",
                                             this_path + "/images/pennovation2.png")
        # We will wait for a mode from the state machine
        self.mode = None
        self.waypoint_list = list(self.planner.waypoints.keys())
        self.explore_target_waypt = self.waypoint_list.copy()

        # Create threads for the different modes
        self.explore_thread = threading.Thread(target=self.exploration_thread)
        self.explore_thread.daemon = True
        self.goto_robot_thread = threading.Thread(target=self.goto_robot_thread)
        self.goto_robot_thread.daemon = True

        # Create subscribers for the state machine topics: goal and coordinates
        rospy.Subscriber("/airrouter/goal", Goal, self.goal_callback)

        # Create a subscriber for the UAV position
        rospy.Subscriber("/airrouter/pose", PointStamped, self.pose_callback)



    def goal_callback(self, data):
        if self.mode is None and data.action == "explore":
            # initial explore
            self.explore_thread.start()

        elif self.mode == "explore" and data.action == "go to robot":
            self.mode = "go to robot"
            # transition to go to robot
            pass
        elif self.mode == "go to robot" and data.action == "explore":
            self.mode = "explore"
            # transition to explore
            pass
        else:
            # unknown transition. Die
            rospy.logerr("Unknown transition from %s to %s", self.mode, data.action)
            rospy.signal_shutdown("Shutting down navigator")

    def pose_callback(self, data):
        self.uav_pose = data

    def send_waypoint_uav(waypoint):
        pass

    def arrived_at_waypoint(self, waypoint):
        pass

    def exploration_thread(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Get the top element on the list as target waypoint
            target = self.explore_target_waypt[0]
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
    Navigator()
    rospy.spin()
