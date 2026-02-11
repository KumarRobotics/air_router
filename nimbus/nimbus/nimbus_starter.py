#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPull

class MissionStarter(Node):
    def __init__(self):
        super().__init__('mission_starter')
        
        # 1. Parameter: How long to wait after launch before doing anything?
        self.declare_parameter('start_delay', 10.0) 
        start_delay_sec = self.get_parameter('start_delay').value

        # 2. Parameter: How many other nodes must be listening?
        self.declare_parameter('expected_subscribers', 1)
        self.target_count = self.get_parameter('expected_subscribers').value
        
        self.mission_topic = '/mavros/mission/waypoints'
        self.pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')
        
        self.get_logger().info(f"Node launched. Waiting {start_delay_sec} seconds before starting checks...")

        # 3. Create a One-Shot Timer for the initial delay
        # This calls 'start_sequence' once after the delay passes
        self._startup_timer = self.create_timer(start_delay_sec, self.start_sequence)
        
        # Placeholder for the checking timer (we don't create it yet)
        self._check_timer = None


    def start_sequence(self):
        """Called once after the initial start_delay."""
        # Destroy the startup timer so it doesn't fire again
        self.destroy_timer(self._startup_timer)
        
        self.get_logger().info(f"Startup delay complete. Waiting for {self.target_count} subscribers...")

        # Now create the timer that checks for subscribers
        self._check_timer = self.create_timer(0.5, self.check_subscribers)


    def check_subscribers(self):
        # Count how many nodes are subscribing to the mission topic
        try:
            subs_count = self.count_subscribers(self.mission_topic)
        except Exception as e:
            self.get_logger().warn(f"Could not count subscribers: {e}")
            return

        # Barrier Check
        if subs_count >= self.target_count:
            self.get_logger().info("Target subscriber count reached! Requesting Mission Pull...")
            self.trigger_pull()


    def trigger_pull(self):
        # Stop checking
        if self._check_timer:
            self._check_timer.cancel()
        
        if not self.pull_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("MAVROS pull service not available!")
            return

        req = WaypointPull.Request()
        future = self.pull_client.call_async(req)
        future.add_done_callback(self.response_callback)


    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Mission pulled successfully! PX4 has {response.wp_received} waypoints.")
            else:
                self.get_logger().warn("Mission pull request failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = MissionStarter()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()