#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Path
import threading
from enum import Enum, auto
from air_router.msg import Goal
import pdb

""" State machine node
The state machine node is responsible for the high level control of the aerial
robot. It will explore the environment for a certain amount of time, then
it will try to find a communication robot. If it finds a communication robot,
it will return to exploration mode.
"""

# Defaults: explore for 5 minutes before trying to find a communication robot
DEFAULT_INITIAL_EXPLORATION_TIME = 1*60
DEFAULT_SHORT_EXPLORATION_TIME = 60//2
DEFAULT_SEARCH_TIME = 1*60

# Defaults: A robot is alive if we communicated in the last 20 minutes
DEFAULT_ALIVE_TIME = 20*60


class Robot:
    """ We create one Robot class per ground robot that we are tracking """
    def __init__(self, robot_name, update_callback, alive_time):
        self.robot_name = robot_name
        self.last_heartbeat = None
        self.last_pose = None
        self.last_pose_ts = None
        self.last_destination = None
        self.last_destination_ts = None
        self.alive_time = alive_time

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
            rospy.loginfo(f"{rospy.get_name()}: {self.robot_name} found")
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
        self.last_destination = last_destination

        # Call the heartbeat function, as we got information from the robot
        self.heartbeat()

    def is_alive(self):
        if self.last_heartbeat is not None:
            return (rospy.get_time() - self.last_heartbeat < self.alive_time)
        return False

    def where_to_find_me(self):
        if (self.last_destination_ts is not None and
                self.last_pose_ts is not None):
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
        idle = auto()
        exploration_initial = auto()
        exploration_short = auto()
        search = auto()
        wait_search = auto()

    def __init__(self):
        rospy.init_node("state_machine")

        # get parameters for the node
        if not rospy.has_param("~initial_exploration_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default initial_exploration_time")
        self.initial_expl_time = rospy.get_param("~initial_exploration_time",
                                                 DEFAULT_INITIAL_EXPLORATION_TIME)
        rospy.loginfo(f"{rospy.get_name()}: initial_exploration_time: {self.initial_expl_time}")

        if not rospy.has_param("~short_exploration_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default short_exploration_time")
        self.short_expl_time = rospy.get_param("~short_exploration_time",
                                               DEFAULT_SHORT_EXPLORATION_TIME)
        rospy.loginfo(f"{rospy.get_name()}: short_exploration_time: {self.short_expl_time}")

        if not rospy.has_param("~search_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default search_time")
        self.search_time = rospy.get_param("~search_time", DEFAULT_SEARCH_TIME)
        rospy.loginfo(f"{rospy.get_name()}: search_time: {self.search_time}")

        if not rospy.has_param("~alive_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default alive_time")
        self.alive_time = rospy.get_param("~alive_time", DEFAULT_ALIVE_TIME)
        rospy.loginfo(f"{rospy.get_name()}: alive_time: {self.alive_time}")

        if not rospy.has_param("~robot_list"):
            rospy.logfatal(f"{rospy.get_name()}: No robot_list parameter")
            rospy.signal_shutdown("No robot_list parameter")
            return

        # Create two topics. One for the goal of the robot
        # ("go to robot", "explore"). If we are in "go to robot"
        # the second topic will contain the coordinates of the robot
        self.goal_pub = rospy.Publisher("/air_router/goal",
                                        Goal, queue_size=1)

        # Subscribe to a topic that starts the state machine
        rospy.Subscriber("/air_router/start", Empty, self.start_callback)
        self.start = False

        # Subscribe to the navigator state
        rospy.Subscriber("/air_router/navigator/state", String,
                         self.update_state)

        # Robot list is a list of comma separated robots. Generate a list
        rlist = rospy.get_param("~robot_list").split(",")
        rlist = [r.strip() for r in rlist]
        assert len(rlist) > 0
        self.robot_list = [Robot(r, self.robot_found_callback, self.alive_time)
                           for r in rlist]
        rospy.loginfo(f"{rospy.get_name()}: Robot list: {', '.join(rlist)}")

        # Create a state variable
        self.state = self.State.idle

        # Timer object stores the current timer
        self.timer = None

        # Did we find a robot during the last search?
        self.found_robot = False

        # Fail robot search at most 5 times
        self.fail_count = 0

        # We are good to go!
        rospy.loginfo(f"{rospy.get_name()}: Started")

        # TODO: use the start condition instead of calling the function to
        # trigger the change to exploration
        rospy.sleep(2)
        self.update_state(None)

    def timer_callback(self, event):
        self.timer = None
        self.update_state("timeout")

    def robot_found_callback(self):
        self.found_robot = True

    def start_callback(self, msg):
        self.start = True

    def set_state(self, state):
        self.state = state

    def state_exploration_short(self):
        # First, cancel the timer
        if self.timer is not None:
            self.timer.shutdown()
        # Go explore
        self.goal_pub.publish(Goal("explore", None))
        self.set_state(self.State.exploration_short)
        rospy.loginfo(f"{rospy.get_name()}: Short Expl - Starting")
        # Timer will be manually started once we resume exploration

    def state_exploration_initial(self):
        # First, cancel the timer
        if self.timer is not None:
            self.timer.shutdown()
        # Go explore
        self.goal_pub.publish(Goal("explore", None))
        self.set_state(self.State.exploration_initial)
        rospy.Timer(rospy.Duration(self.initial_expl_time),
                    self.timer_callback, oneshot=True)
        rospy.loginfo(f"{rospy.get_name()}: Initial Expl - Starting")

    def state_search(self):
        if self.timer is not None:
            self.timer.shutdown()
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
            if self.fail_count > 5:
                rospy.logfatal(f"{rospy.get_name()}: Search - " +
                               "Too many failed searches, shutting down")
                rospy.signal_shutdown("Too many failed searches")
                return
            self.fail_count += 1
            rospy.logwarn(f"{rospy.get_name()}: No robot to search - Count: {self.fail_count}")
            self.state_exploration_short()
            return

        # We did find a robot, go search for it
        rospy.logwarn(f"{rospy.get_name()}: Search - " +
                      "finding robot {robot_to_find.robot_name}")
        self.set_state(self.State.search)
        robot_to_find.robot_searched = True
        goal = robot_to_find.where_to_find_me()
        rospy.Timer(rospy.Duration(self.search_time), self.timer_callback,
                    oneshot=True)
        self.goal_pub.publish(Goal("go to robot", goal))

    def state_wait_after_search(self):
        # We are just waiting, do nothing
        rospy.loginfo(f"{rospy.get_name()}: Search Wait - Starting")
        self.set_state(self.State.wait_search)

    def update_state(self, msg):
        if self.state == self.State.idle:
            # if self.start:
            #     self.state_exploration_initial()
            self.state_exploration_initial()
        elif self.state == self.State.exploration_initial:
            if msg == "timeout" or msg == "explore_end":
                self.state_search()
        elif self.state == self.State.exploration_short:
            # Wait for the go_to_target_end to start timer
            if msg == "go_to_target_end":
                rospy.Timer(rospy.Duration(self.short_expl_time),
                            self.timer_callback, oneshot=True)
            if msg == "timeout" or msg == "explore_end":
                self.state_search()
        elif self.state == self.State.search:
            if msg == "go_to_target_end":
                self.state_wait_after_search()
            if msg == "timeout":
                self.state_exploration_short()
        elif self.state == self.State.wait_search:
            if msg == "timeout":
                self.state_exploration_short()


if __name__ == "__main__":
    StateMachine()
    rospy.spin()
