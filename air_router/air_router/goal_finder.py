#!/usr/bin/env python3
import pdb
import random
import threading
from enum import Enum, auto

import zmq
# from air_router.msg import Goal
from router_interfaces.msg import Goal
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, String
from builtin_interfaces.msg import Time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock


class ZMQ_thread(threading.Thread):
    def __init__(self, outer):
        super().__init__()
        self.context = zmq.Context()
        self.zac_socket = self.context.socket(zmq.PULL)
        self.zac_socket.connect("tcp://localhost:6555")
        # self.det_socket = self.context.socket(zmq.PUSH)
        # self.det_socket.bind("tcp://*:6556")
        # self.stop_event = stop_event
        self.outer = outer

    def run(self):
        while not self.outer.stop_event.is_set():
            # self.get_logger().info(f"Thread - Waiting for msg")
            msg = self.zac_socket.recv_pyobj()
            point = PointStamped()
            timestamp = msg["timestamp"]
            secs = int(timestamp)
            nsecs = int((timestamp - secs) * 1e9)
            point.header.stamp.secs = secs
            point.header.stamp.nsecs = nsecs
            point.header.frame_id = "Good Soup Frame"
            point.point.x = msg["x"]
            point.point.y = msg["y"]
            self.outer.update_goal(point)
            print("Good soup transmission")


class GoalFinder(Node):
    class State(Enum):
        init = auto()
        idle = auto()
        search = ()

    def __init__(self):
        super().__init__(f"goal_finder{random.randint(0, 999)}")

        self.goal_pub = self.create_publisher(Goal, 'air_router/goal', 1)

        # Create two topics. One for the goal of the robot
        # ("go to robot", "explore"). If we are in "go to robot"
        # the second topic will contain the coordinates of the robot

        # Subscribe to the navigator state
        self.create_subscription(String, 'air_router/navigator/state', self.update_state, 10)

        header = Header()
        header.stamp = Clock().now().to_msg()  # ROS 2 time message
        header.frame_id = "quad"
        point = Point()
        point.x = -37.0
        point.y = 96.0
        point.z = 50.0
        pStamped = PointStamped()
        pStamped.header = header
        pStamped.point = point
        self.current_goal = pStamped

        # This is where we get the goal position
        self.get_logger().info(f"{self.get_name()}: Started")

        # Set state
        self.lock = threading.Lock()
        self.state = self.State.init

        # Check the parameter use_zmq. If true, we use zmq to receive the goal.
        # Otherwise we use a ros topic
        if self.has_parameter('use_zmq'):
            self.get_logger().warn(f"{self.get_name()}: Using ZMQ to get the target goal")
            self.stop_event = threading.Event()
            self.socket_thread = ZMQ_thread(self)
            self.socket_thread.run()

        else:
            self.get_logger().warn(f"{self.get_name()}: Using topic to get the target goal")
            self.create_subscription(PointStamped, 'air_router/goal_finder/goal', self.update_goal, 1)

    def state_search(self):
        self.set_state(self.State.search)
        # We did find a robot, go search for it
        # self.reset_timer()
        self.get_logger().warn(f"{self.get_name()}: Searching for \n{self.current_goal}")
        new_goal = Goal()
        new_goal.action = "go to robot"
        new_goal.goal = self.current_goal
        self.goal_pub.publish(new_goal)

    def update_goal(self, msg):
        self.current_goal = msg
        # preempt the state machine
        self.update_state(String("new_goal"))

    def set_state(self, state):
        self.state = state

    def update_state(self, msg):

        def wrong_message(msg):
            self.get_logger().error(
                f"{self.get_name()}: "
                + f"Unexpected message {msg} in state {self.state}"
            )
            rclpy.shutdown()

        msg = msg.data

        if msg == "go_to_target" or msg == "explore" or msg == "transition":
            # Ignore these messages from the navigator
            return

        self.lock.acquire()
        if self.state == self.State.init:
            if msg == "init":
                self.get_logger().info(f"{self.get_name()}: Init - Starting")
                self.state_search()
            else:
                wrong_message(msg)

        elif self.state == self.State.search:
            if msg == "go_to_target_end":
                self.get_logger().info(f"{self.get_name()}: Search - Reached target")
                self.set_state(self.State.idle)
            elif msg == "new_goal":
                self.get_logger().info(f"{self.get_name()}: Search - New goal received")
                self.state_search()
            else:
                wrong_message(msg)
        elif self.state == self.State.idle:
            if msg == "new_goal":
                self.get_logger().info(f"{self.get_name()}: Idle - New goal received")
                self.state_search()
            else:
                wrong_message(msg)
        else:
            wrong_message(msg)
        self.lock.release()

def main(args=None):
    rclpy.init(args=args)
    goal_node = GoalFinder()

    print(f"\tSpinning node...")

    rclpy.spin(goal_node)

    nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
