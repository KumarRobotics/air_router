#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, Float64MultiArray, String
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Path
import threading
from enum import Enum
from air_router.msg import Goal
import pdb
import numpy as np


# List of all the robots
JACKALS = ["husky1", "husky2", "husky3"]

# Explore for 5 minutes before trying to find a communication robot
INITIAL_EXPLORATION_TIME_S = 1*60
SHORT_EXPLORATION_TIME_S = 60//2
SEARCH_TIME_S = 1*60

# A robot is alive if we communicated in the last 20 minutes
ALIVE_TIME_S = 20*60


class Robot:
    def __init__(self, robot_name, update_callback):
        self.robot_name = robot_name
        self.last_heartbeat = None
        self.last_pose = None
        self.last_pose_ts = None
        self.last_destination = None
        self.last_destination_ts = None

        # Create a ROS subscriber for the transmit heartbeat. The database will
        # transmit a hearbeat every time a communication happens with the robot

        # Create a subscriber for the robot pose
        rospy.Subscriber(f"{self.robot_name}/top_down_render/pose_est",
                         PoseWithCovarianceStamped, self.pose_callback)

        # Create a subscriber for the robot destination
        rospy.Subscriber(f"{self.robot_name}/spomp_global/path_viz",
                         Path, self.destination_callback)

        # Update callback is a function pointer that will be called upon an
        # update
        self.update_callback = update_callback
        self.robot_searched = False

    def heartbeat(self):
        # Set current time as the last heartbeat
        self.last_heartbeat = rospy.get_time()
        if self.robot_searched:
            self.robot_searched = False
            self.update_callback()

    def pose_callback(self, msg):
        # Call the heartbeat function, as we got information from the robot
        self.heartbeat()
        # Set the current pose as a PointStamped
        self.last_pose = PointStamped()
        self.last_pose.header = msg.header
        self.last_pose.point = msg.pose.pose.position
        self.last_pose.header.frame_id = self.robot_name
        self.last_pose_ts = rospy.get_time()

    def destination_callback(self, msg):
        # msg is a Path, we only care about the last point.
        # Store as a PointStamped
        last_destination = PointStamped()
        last_destination.header = msg.header
        # If the robot reaches a destination and the quad is not notified of
        # that destination, just send the current pose as destination
        if len(msg.poses) == 0:
            if self.last_pose is not None:
                last_destination.point = self.last_pose.point
            else:
                return
        else:
            last_destination.point = msg.poses[-1].pose.position
        last_destination.header.frame_id = self.robot_name
        last_destination_ts = rospy.get_time()
        self.last_destination = last_destination

        # Call the heartbeat function, as we got information from the robot
        self.heartbeat()

    def is_alive(self):
        if self.last_heartbeat is not None:
            return (rospy.get_time() - self.last_heartbeat < ALIVE_TIME_S)
        return False

    def where_to_find_me(self):
        if self.last_destination_ts is not None and self.last_pose_ts is not None:
            if self.last_destination_ts >= self.last_pose_ts:
                return self.last_destination
            else:
                return self.last_pose
        elif self.last_destination_ts is not None:
            return self.last_destination
        elif self.last_pose_ts:
            return self.last_pose
        return None


class StateMachine:
    class State(Enum):
        idle = 0
        exploration_initial = 1
        exploration_short = 2
        search = 3
        wait_search = 4

    def __init__(self, robots):
        assert isinstance(robots, list)
        rospy.init_node("state_machine")
        # Create two topics. One for the goal of the robot
        # ("go to robot", "explore"). If we are in "go to robot"
        # the second topic will contain the coordinates of the robot
        self.goal_pub = rospy.Publisher("/air_router/goal",
                                        Goal, queue_size=1)

        # Subscribe to a topic that starts the state machine
        rospy.Subscriber("/air_router/start", Empty, self.start_callback)
        self.start = False

        # List of robots for round robin search
        self.robot_list = [Robot(robot, self.robot_found_callback) for robot in robots]

        # Create a variable protected by a lock for the state
        self.state = self.State.idle
        self.state_lock = threading.Lock()

        # timestamp when we started the last exploration or search
        self.last_ts = None

        # Did we find a robot during the last search?
        self.found_robot = False

        # Create a timer to update the state machine regularly
        self.timer = rospy.Timer(rospy.Duration(1), self.update_state)

    def robot_found_callback(self):
        self.found_robot = True

    def start_callback(self, msg):
        self.start = True

    def set_state(self, state):
        self.state_lock.acquire()
        self.state = state
        self.state_lock.release()

    def state_exploration_short(self):
        self.last_ts = rospy.get_time()
        self.goal_pub.publish(Goal("explore", None))
        self.set_state(self.State.exploration_short)
        rospy.loginfo("EXPLORATION: Starting")

    def state_exploration_initial(self):
        self.last_ts = rospy.get_time()
        self.goal_pub.publish(Goal("explore", None))
        self.set_state(self.State.exploration_initial)
        rospy.loginfo("EXPLORATION: Initial")

    def state_search(self):
        rospy.loginfo("SEARCH: Starting")
        self.found_robot = False
        # Find the robot to search in a round robin fashion
        robot_to_find = None
        for _ in range(len(self.robot_list)):
            top_robot = self.robot_list[0]
            # rotate the list
            self.robot_list = self.robot_list[1:] + [top_robot]

            # Check if the robot is alive and has a location estimation
            if top_robot.is_alive() and top_robot.where_to_find_me():
                robot_to_find = top_robot
                break

        # if we did not find a robot, go back to exploration short
        # but be angry about it
        if robot_to_find is None:
            rospy.logerr("SEARCH: Did not find a robot to search for")
            return

        rospy.loginfo(f"SEARCH: Finding robot: {robot_to_find.robot_name}")
        # We did find a robot, go search for it
        self.set_state(self.State.search)
        robot_to_find.robot_searched = True
        goal = robot_to_find.where_to_find_me()
        self.goal_pub.publish(Goal("go to robot", goal))
        self.last_ts = rospy.get_time()

    def state_wait_search(self):
        # We are just waiting, do nothing
        self.set_state(self.State.wait_search)
        rospy.loginfo("WAIT: waiting for robot search")

    def update_state(self, event):
        if self.state == self.State.idle:
            # if self.start:
            #     self.state_exploration_initial()
            self.state_exploration_initial()
        elif self.state == self.State.exploration_initial:
            if rospy.get_time() - self.last_ts > INITIAL_EXPLORATION_TIME_S:
                self.state_search()
        elif self.state == self.State.exploration_short:
            if rospy.get_time() - self.last_ts > SHORT_EXPLORATION_TIME_S:
                self.state_search()
        elif self.state == self.State.search:
            self.state_wait_search()
        elif self.state == self.State.wait_search:
            if self.found_robot or (rospy.get_time() - self.last_ts > SEARCH_TIME_S):
                self.state_exploration_short()


if __name__ == "__main__":
    StateMachine(JACKALS)
    rospy.spin()
