#!/usr/bin/env python3
import rclpy, csv, os
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("waypoint_recorder")
        
        # Get filename to save waypoint
        self.declare_parameter("raw_file", "raw_waypoints.csv")
        
        # Logic to find the 'src' directory
        install_prefix = os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0]
        self.pkg_path = os.path.abspath(os.path.join(install_prefix, '..', '..', 'src', 'pure_pursuit', 'waypoints'))
        os.makedirs(self.pkg_path, exist_ok=True)
        
        # Create waypoint file
        self.filename = self.get_parameter("raw_file").value
        self.full_path = os.path.join(self.pkg_path, self.filename)
        
        # Pub and sub for recording waypoints
        self.raw_points = []
        self.sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.record_cb, 10)
        self.viz_pub = self.create_publisher(MarkerArray, "/raw_markers", 10)
        
        self.get_logger().info(f"Waypoint Recorder Active. Saving to: {self.full_path}")
        self.get_logger().info("Use '2D Pose Estimate' in RViz to drop points.")

    def record_cb(self, msg):
        """Records point from foxglove click, saves to disk, and updates visualization."""
        p = msg.pose.pose.position
        self.raw_points.append([p.x, p.y])
        
        # Save immediately to ensure data persistence
        try:
            with open(self.full_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y"])
                writer.writerows(self.raw_points)
            self.get_logger().info(f"Point recorded: [{p.x:.2f}, {p.y:.2f}]. Total: {len(self.raw_points)}")
        except Exception as e:
            self.get_logger().error(f"Failed to save: {e}")

        self.publish_viz()

    def publish_viz(self):
        """Publishes MarkerArray so you can see your path being drawn in real-time."""
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.raw_points):
            # Create marker for specific waypoint
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.id, m.type = i, Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = x, y
            m.scale.x = m.scale.y = m.scale.z = 0.25
            # Make point bright green
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            ma.markers.append(m)

        self.viz_pub.publish(ma)

def main():
    rclpy.init(); rclpy.spin(WaypointRecorder()); rclpy.shutdown()

if __name__ == "__main__":
    main()