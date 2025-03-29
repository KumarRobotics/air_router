#!/usr/bin/env python3
import pdb
import threading
from enum import Enum, auto

import rospy
from air_router.msg import Goal
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, String, Time


class GoalFinder:
    class State(Enum):
        init = auto()
        idle = auto()
        search = ()

    def __init__(self):
        rospy.init_node("goal_finder")

        # Create two topics. One for the goal of the robot
        # ("go to robot", "explore"). If we are in "go to robot"
        # the second topic will contain the coordinates of the robot
        self.goal_pub = rospy.Publisher("air_router/goal", Goal, queue_size=1)

        # Subscribe to the navigator state
        rospy.Subscriber("air_router/navigator/state", String, self.update_state)

        header = Header(0, rospy.get_rostime(), "quad")
        point = Point(0, 0, 0)
        self.current_goal = PointStamped(header, point)

        # This is where we get the goal position
        rospy.Subscriber("air_router/goal_finder/goal", PointStamped, self.update_goal)
        rospy.loginfo(f"{rospy.get_name()}: Started")

        # Set state
        self.lock = threading.Lock()
        self.state = self.State.init

    def state_search(self):
        self.set_state(self.State.search)
        # We did find a robot, go search for it
        # self.reset_timer()
        rospy.logwarn(f"{rospy.get_name()}: Searching for \n{self.current_goal}")
        self.goal_pub.publish(Goal("go to robot", self.current_goal))

    def update_goal(self, msg):
        self.current_goal = msg
        # preempt the state machine
        self.update_state(String("new_goal"))

    def set_state(self, state):
        self.state = state

    def update_state(self, msg):
        def wrong_message(msg):
            rospy.logerr(
                f"{rospy.get_name()}: "
                + f"Unexpected message {msg} in state {self.state}"
            )
            rospy.signal_shutdown(f"Unexpected message {msg}")
            rospy.spin()

        msg = msg.data

        if msg == "go_to_target" or msg == "explore" or msg == "transition":
            # Ignore these messages from the navigator
            return

        self.lock.acquire()
        if self.state == self.State.init:
            if msg == "init":
                rospy.loginfo(f"{rospy.get_name()}: Init - Starting")
                self.state_search()
            else:
                pdb.set_trace()
                wrong_message(msg)

        elif self.state == self.State.search:
            if msg == "go_to_target_end":
                rospy.loginfo(f"{rospy.get_name()}: Search - Reached target")
                self.set_state(self.State.idle)
            elif msg == "new_goal":
                rospy.loginfo(f"{rospy.get_name()}: Search - New goal received")
                self.state_search()
            else:
                wrong_message(msg)
        elif self.state == self.State.idle:
            if msg == "new_goal":
                rospy.loginfo(f"{rospy.get_name()}: Idle - New goal received")
                self.state_search()
            else:
                wrong_message(msg)
        else:
            wrong_message(msg)
        self.lock.release()


if __name__ == "__main__":
    GoalFinder()
    rospy.spin()
