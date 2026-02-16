#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPull


class MissionStarter(Node):
    def __init__(self):
        super().__init__('nimbus_starter')
        
        self.declare_parameter('start_delay', 10.0) 
        start_delay_sec = self.get_parameter('start_delay').value
        
        self.declare_parameter('expected_subscribers', 2)
        self.target_count = self.get_parameter('expected_subscribers').value
        
        self.mission_topic = '/mavros/mission/waypoints'
        
        # Clients
        self.ms_pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')
        self.gf_pull_client = self.create_client(WaypointPull, '/mavros/geofence/pull')
        
        self.get_logger().info(f"Node launched. Waiting {start_delay_sec}s...")
        
        self._startup_timer = self.create_timer(start_delay_sec, self.start_sequence)
        self._check_timer = None


    def start_sequence(self):
        self.destroy_timer(self._startup_timer)
        self.get_logger().info(f"Startup delay complete. Waiting for {self.target_count} subscribers...")
        self._check_timer = self.create_timer(0.5, self.check_subscribers)


    def check_subscribers(self):
        try:
            subs_count = self.count_subscribers(self.mission_topic)
        except Exception as e:
            self.get_logger().warn(f"Could not count subscribers: {e}")
            return
        
        if subs_count >= self.target_count:
            self.get_logger().info("Target count reached! Starting Pull Sequence...")
            # Stop the timer..
            self._check_timer.cancel()
            # Pull mission waypoints
            self.trigger_mission_pull()
        else:
            self.get_logger().info(f"Waiting for {self.target_count} subscribers...")


    # --- Pull Mission ---
    def trigger_mission_pull(self):
        # Check if service is available
        if not self.ms_pull_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Mission service unavailable!")
            return
        
        # Pull waypoints
        future = self.ms_pull_client.call_async(WaypointPull.Request())
        future.add_done_callback(self.mission_response_callback)


    def mission_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Mission waypoint pull succeeded: {response.wp_received} waypoints")
            else:
                self.get_logger().warn("Mission waypoint pull failed!")
        except Exception as e:
            self.get_logger().error(f"Mission waypoint pull call failed: {e}")
        
        # Try to pull geofence
        self.trigger_geofence_pull()


    # --- Pull Geofence ---
    def trigger_geofence_pull(self):
        # Check if service is available
        if not self.gf_pull_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Geofence service unavailable!")
            return
        
        # Pull geofence
        future = self.gf_pull_client.call_async(WaypointPull.Request())
        future.add_done_callback(self.geofence_response_callback)


    def geofence_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Geofence pull succeeded: {response.wp_received} items.")
            else:
                self.get_logger().warn("Geofence pull failed!")
        except Exception as e:
            self.get_logger().error(f"Geofence pull call failed: {e}")
        
        self.get_logger().info("Start-up Complete. Exiting.")
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