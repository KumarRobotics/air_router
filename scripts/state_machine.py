#!/usr/bin/env python3
import rospy
from std_msgs.msg import Time, String
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Path
from enum import Enum, auto
from air_router.msg import Goal
import threading
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
DEFAULT_AFTER_SEARCH_TIME = 20
DEFAULT_TRACKING_TIME = 60

# Defaults: A robot is alive if we communicated in the last 20 minutes
DEFAULT_ALIVE_TIME = 20*60

# Defaults: we will prefer searching for the robot on its target position
# instead of its current position if the target position is at most two
# minutes older than the current position.

DEFAULT_PREFER_TARGET_TIME = 2*60


class Node:
    """ The Node class encompases both ground robots and the basestation """
    def __init__(self, node_name, update_callback, alive_time):
        # Check input arguments
        assert isinstance(node_name, str)
        assert callable(update_callback)
        assert isinstance(alive_time, int) or isinstance(alive_time, float)

        self.node_name = node_name
        self.alive_time = alive_time

        # We will use the last heartbeat to determine if the node is alive,
        # and to determine if we found it (when it is updated)self.
        self.last_heartbeat = None

        # Create a subscriber for the distributed database end of transmission
        # _when the quad finishes the transmission_ to the clients (server side)
        rospy.Subscriber(f"ddb/server_sync_complete/{self.node_name}",
                         Time, self.sync_complete_callback)

        self.node_searched = False

        # Update callback is a function pointer that will be called upon an
        # update
        self.update_callback = update_callback

    def heartbeat(self):
        # Set current time as the last heartbeat
        self.last_heartbeat = rospy.get_time()

    def sync_complete_callback(self, msg):
        # Call the heartbeat function, as we got information from the robot
        self.heartbeat()
        # Signal the state machine that sync completed
        if self.node_searched:
            rospy.logwarn(f"{rospy.get_name()}: {self.node_name} " +
                          "sync complete")
            self.update_callback()

    def is_alive(self):
        if self.last_heartbeat is not None:
            return (rospy.get_time() - self.last_heartbeat < self.alive_time)
        return False

    def where_to_find_me(self):
        # By default, send an empty point
        point = PointStamped()
        point.header.frame_id = self.node_name
        return point

    def set_node_search_state(self, state):
        assert isinstance(state, bool)
        rospy.logwarn(f"{rospy.get_name()}: {self.node_name} " +
                      f"search state set to {state}")
        self.node_searched = state


class Robot(Node):
    def __init__(self, robot_name, update_callback, pose_updated_callback,
                 alive_time, recent_pose_time_threshold):
        super().__init__(robot_name, update_callback, alive_time)
        assert isinstance(recent_pose_time_threshold, int) or \
            isinstance(recent_pose_time_threshold, float)
        self.last_pose = None
        self.last_pose_ts = None
        self.last_destination = None
        self.last_destination_ts = None
        self.recent_pose_time_threshold = recent_pose_time_threshold
        self.robot_name = robot_name

        # Create a subscriber for the robot pose
        rospy.Subscriber(f"{self.robot_name}/top_down_render/pose_est",
                         PoseWithCovarianceStamped, self.pose_callback)

        # Create a subscriber for the robot destination
        rospy.Subscriber(f"{self.robot_name}/spomp_global/path_viz",
                         Path, self.destination_callback)

        # Create a callback for the pose updated
        self.pose_updated_callback = pose_updated_callback

    def pose_callback(self, msg):
        # Call the heartbeat function, as we got information from the robot
        self.heartbeat()
        # Set the current pose as a PointStamped
        self.last_pose = PointStamped()
        self.last_pose.header = msg.header
        self.last_pose.point = msg.pose.pose.position
        self.last_pose.header.frame_id = self.robot_name
        self.last_pose_ts = rospy.get_time()
        if self.node_searched:
            self.pose_updated_callback()

    def destination_callback(self, msg):
        # Call the heartbeat function, as we got information from the robot
        self.heartbeat()
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
        self.last_destination_ts = rospy.get_time()

    def where_to_find_me(self):
        if self.last_pose_ts is not None and \
                (rospy.get_time() - self.last_pose_ts < self.recent_pose_time_threshold):
            # Print time with two decimals
            rospy.logdebug(f"{self.robot_name} going to last pose (updated " +
                           f"{rospy.get_time() - self.last_pose_ts:.2f}s ago)")
            return self.last_pose
        elif self.last_destination_ts is not None:
            rospy.logdebug(f"{self.robot_name} going to last destination")
            return self.last_destination
        elif self.last_pose_ts is not None:
            rospy.logdebug(f"{self.robot_name} going to last pose" +
                           "(no destination)")
            return self.last_pose
        return None


class StateMachine:
    class State(Enum):
        idle = auto()
        exploration_initial = auto()
        exploration_short = auto()
        search = auto()
        wait_search = auto()
        tracking = auto()

    def __init__(self):
        rospy.init_node("state_machine")

        # get parameters for the node
        if not rospy.has_param("~initial_exploration_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default" +
                          "initial_exploration_time")
        self.initial_expl_time = rospy.get_param("~initial_exploration_time",
                                                 DEFAULT_INITIAL_EXPLORATION_TIME)
        rospy.loginfo(f"{rospy.get_name()}: initial_exploration_time: " +
                      f"{self.initial_expl_time}")

        if not rospy.has_param("~short_exploration_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default" +
                          "short_exploration_time")
        self.short_expl_time = rospy.get_param("~short_exploration_time",
                                               DEFAULT_SHORT_EXPLORATION_TIME)
        rospy.loginfo(f"{rospy.get_name()}: short_exploration_time: " +
                      f"{self.short_expl_time}")

        if not rospy.has_param("~after_search_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default after search_time")
        self.after_search_time = rospy.get_param("~after_search_time",
                                                 DEFAULT_AFTER_SEARCH_TIME)
        rospy.loginfo(f"{rospy.get_name()}: after_search_time: " +
                      f"{self.after_search_time}")

        if not rospy.has_param("~tracking_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default tracking_time")
        self.tracking_time = rospy.get_param("~tracking_time",
                                             DEFAULT_TRACKING_TIME)
        rospy.loginfo(f"{rospy.get_name()}: tracking_time: " +
                      f"{self.tracking_time}")

        if not rospy.has_param("~alive_time"):
            rospy.logwarn(f"{rospy.get_name()}: Default alive_time")
        self.alive_time = rospy.get_param("~alive_time", DEFAULT_ALIVE_TIME)
        rospy.loginfo(f"{rospy.get_name()}: alive_time: {self.alive_time}")

        if not rospy.has_param("~recent_pose_time_threshold"):
            rospy.logwarn(f"{rospy.get_name()}: Default" +
                          "recent_pose_time_threshold")
        self.recent_pose_time_threshold = rospy.get_param("~recent_pose_time_threshold", 30)
        rospy.loginfo(f"{rospy.get_name()}: recent_pose_time_threshold:" +
                      f"{self.recent_pose_time_threshold}")

        if not rospy.has_param("~robot_list"):
            rospy.logfatal(f"{rospy.get_name()}: No robot_list parameter")
            rospy.signal_shutdown("No robot_list parameter")
            return

        # Create two topics. One for the goal of the robot
        # ("go to robot", "explore"). If we are in "go to robot"
        # the second topic will contain the coordinates of the robot
        self.goal_pub = rospy.Publisher("air_router/goal",
                                        Goal, queue_size=1)

        # Subscribe to the navigator state
        rospy.Subscriber("air_router/navigator/state", String,
                         self.update_state)

        # Robot list is a list of comma separated robots. Generate a list
        rlist = rospy.get_param("~robot_list").split(",")
        rlist = [r.strip() for r in rlist]
        assert len(rlist) > 0
        self.robot_list = [Robot(r, self.transmission_complete_callback,
                                 self.pose_updated_callback,
                                 self.alive_time,
                                 self.recent_pose_time_threshold)
                           for r in rlist]

        # Add the basestation as a fake robot
        basestation = Node("basestation",
                           self.transmission_complete_callback,
                           self.alive_time)
        self.robot_list.append(basestation)

        rospy.loginfo(f"{rospy.get_name()}: Node list: {', '.join(rlist)}")

        # Create a state variable
        self.state = self.State.idle

        # Timer object stores the current timer
        self.timer = None

        # Current target robot
        self.robot_target = None
        self.robot_position_target = None

        # Did we finish the exploration?
        self.exploration_finished = False

        # Create a lock for the state machine changes
        self.lock = threading.Lock()

        # We are good to go! The trigger from the init state will be done
        # by the navigator node
        rospy.loginfo(f"{rospy.get_name()}: Started")

    def timer_callback(self, event):
        self.timer = None
        self.update_state(String("timeout"))

    def set_timer(self, duration):
        self.reset_timer()
        self.timer = rospy.Timer(rospy.Duration(duration), self.timer_callback,
                                 oneshot=True)

    def reset_timer(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None

    def transmission_complete_callback(self):
        if self.state == self.State.search or \
                self.state == self.State.wait_search or \
                self.state == self.State.tracking:
            self.update_state(String("transmission_complete"))

    def pose_updated_callback(self):
        if self.state == self.State.search or \
                self.state == self.State.wait_search or \
                self.state == self.State.tracking:
            self.update_state(String("pose_updated"))

    def set_state(self, state):
        self.state = state

    def find_target(self):
        # Find the robot to search in a round robin fashion
        robot_to_find = None
        for _ in range(len(self.robot_list)):
            top_robot = self.robot_list[0]
            # rotate the list
            self.robot_list = self.robot_list[1:] + [top_robot]

            # Check if the robot is alive and has a location estimation
            where_is = top_robot.where_to_find_me()
            if top_robot.is_alive() and where_is is not None:
                robot_to_find = top_robot
                break

        # Check if we found a robot
        if robot_to_find is None:
            rospy.logwarn(f"{rospy.get_name()}: No target to search")
            self.robot_target = None
            self.robot_position_target = None
        else:
            self.robot_target = robot_to_find
            self.robot_position_target = where_is
        return self.robot_target

    def state_exploration_initial(self):
        self.set_state(self.State.exploration_initial)
        # Go explore
        self.set_timer(self.initial_expl_time)
        self.goal_pub.publish(Goal("explore", None))
        rospy.loginfo(f"{rospy.get_name()}: Initial Expl - Starting")

    def state_exploration_short(self):
        self.set_state(self.State.exploration_short)
        # First, cancel the timer
        self.reset_timer()
        # Go explore
        self.goal_pub.publish(Goal("explore", None))
        rospy.loginfo(f"{rospy.get_name()}: Short Expl - Starting")
        # Timer will be manually started once we resume exploration

    def state_search(self):
        self.set_state(self.State.search)
        # We did find a robot, go search for it
        self.reset_timer()
        rospy.logwarn(f"{rospy.get_name()}: Search - " +
                      f"finding target {self.robot_target.node_name}")
        self.robot_target.set_node_search_state(True)
        self.goal_pub.publish(Goal("go to robot", self.robot_position_target))

    def state_wait_after_search(self):
        self.set_state(self.State.wait_search)
        rospy.loginfo(f"{rospy.get_name()}: Search - Reached Waypoint. " +
                      "Waiting.")
        # We are just waiting, set a timer and do nothing.
        self.set_timer(self.after_search_time)

    def state_tracking(self):
        # Track the robot position to improve accuracy
        if (self.state != self.State.tracking):
            # This is the first execution, set timer
            self.set_timer(self.tracking_time)
        self.set_state(self.State.tracking)
        self.robot_position_target = self.robot_target.where_to_find_me()
        self.goal_pub.publish(Goal("go to robot", self.robot_position_target))
        rospy.loginfo(f"{rospy.get_name()}: Search - Updated Tracking")

    def update_state(self, msg):
        def wrong_message(msg):
            rospy.logerr(f"{rospy.get_name()}: " +
                         f"Unexpected message {msg} in state {self.state}")
            rospy.signal_shutdown(f"Unexpected message {msg}")
            rospy.spin()

        msg = msg.data
        if msg == "go_to_target" or msg == "explore" or msg == "transition":
            # Ignore these messages from navigator
            return

        self.lock.acquire()
        if self.state == self.State.idle:
            if msg == "init":
                self.state_exploration_initial()
            else:
                wrong_message(msg)
        elif self.state == self.State.exploration_initial or \
                self.state == self.State.exploration_short:
            if msg == "go_to_target_end":
                self.set_timer(self.short_expl_time)
            elif msg == "timeout":
                if self.find_target() is not None:
                    self.state_search()
                else:
                    # Set the timer again
                    rospy.logwarn(f"{rospy.get_name()}: " +
                                  "Exploration finished, " +
                                  "but no target found. Exploring again")
                    self.set_timer(self.short_expl_time)
            elif msg == "explore_end":
                self.exploration_finished = True
                while self.find_target() is None:
                    rospy.logwarn(f"{rospy.get_name()}: " +
                                  "Exploration ended and no target found. "
                                  "Trying again.")
                    rospy.sleep(1)
                self.state_search()
            else:
                wrong_message(msg)

        elif self.state == self.State.search:
            if msg == "go_to_target_end":
                rospy.loginfo(f"{rospy.get_name()}: Search - " +
                              f"Reached target {self.robot_target.node_name}")
                self.state_wait_after_search()
            elif msg == "transmission_complete":
                self.robot_target.set_node_search_state(False)
                if not self.exploration_finished:
                    self.state_exploration_short()
                else:
                    while self.find_target() is None:
                        rospy.logwarn(f"{rospy.get_name()}: " +
                                      "Exploration ended and no target found. "
                                      "Trying again.")
                        rospy.sleep(1)
                    self.state_search()
            elif msg == "pose_updated":
                self.state_tracking()
            else:
                wrong_message(msg)

        elif self.state == self.State.tracking or \
                self.state == self.State.wait_search:
            if msg == "go_to_target_end":
                # Do nothing when we reach the target
                pass
            elif msg == "pose_updated":
                # Relaunch the tracking if we get a new pose update
                self.state_tracking()
            elif msg == "transmission_complete" or msg == "timeout":
                self.robot_target.set_node_search_state(False)
                if not self.exploration_finished:
                    self.state_exploration_short()
                else:
                    while self.find_target() is None:
                        rospy.logwarn(f"{rospy.get_name()}: " +
                                      "Exploration ended and no target found. "
                                      "Trying again.")
                        rospy.sleep(1)
                    self.state_search()
            else:
                wrong_message(msg)
        else:
            wrong_message(msg)
        self.lock.release()


if __name__ == "__main__":
    StateMachine()
    rospy.spin()
