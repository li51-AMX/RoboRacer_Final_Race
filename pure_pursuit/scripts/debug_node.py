#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

class PPVisualizer(Node):
    def __init__(self):
        super().__init__('pp_visualizer')
        # Create publishers and subcribers for debug markers
        self.create_subscription(Path, '/debug/waypoints_path', self.path_cb, 10)
        self.create_subscription(PointStamped, '/debug/target_wp', self.target_cb, 10)
        self.marker_pub = self.create_publisher(Marker, '/visuals/target_marker', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/visuals/path_marker', 10)

    def path_cb(self, msg):
        # Create a path marker for the spline the pure_pursuit node generates from the base waypoints
        m = Marker()
        m.header.frame_id = "map"
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.id = 100
        m.scale.x = 0.08
        m.color.g, m.color.a = 1.0, 0.6
        m.points = [p.pose.position for p in msg.poses]
        self.path_marker_pub.publish(m)

    def target_cb(self, msg):
        # Create a red marker at the position of the waypoint the pure_pursuit node is targeting
        m = Marker()
        m.header = msg.header
        m.type = Marker.SPHERE
        m.id = 200
        m.pose.position = msg.point
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.r, m.color.g, m.color.a = 1.0, 0.2, 1.0
        self.marker_pub.publish(m)

def main():
    rclpy.init(); rclpy.spin(PPVisualizer()); rclpy.shutdown()

if __name__ == "__main__":
    main()