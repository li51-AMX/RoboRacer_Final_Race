#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Declare Parameters
        self.declare_parameters(namespace='', parameters=[
            ('detect_distance', 2.3),      # Max range to consider
            ('zone_angle_deg', 18.0),      # Scan width (0 to 18 left, 0 to 18 right)
            ('side_sensitivity', 0.01),    # How much we ignore side-hits (0.1 = mostly ignore, 1.0 = full push)
            ('steer_aggression', 2.0),     # Multiplier for the final steering output
            ('speed_aggression', 2.7),     # How fast speed drops with an obstacle in the way
            ('max_speed', 6),
            ('min_speed', 0),
            ('smoothing_alpha', 0.5)      # Higher = more responsive, Lower = smoother
        ])
        
        self.current_steer = 0.0
        self.current_speed = self.p('max_speed')
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.adj_pub = self.create_publisher(Float32MultiArray, '/obstacle_adjustments', 10)

    def p(self, name):
        return self.get_parameter(name).value

    def get_weighted_score(self, msg, start_idx, end_idx, center_idx):
        """Calculates a 0.0-1.0 threat score using a weighted average of all rays."""
        detect_dist = self.p('detect_distance')
        side_weight = self.p('side_sensitivity')
        zone_rad = np.deg2rad(self.p('zone_angle_deg'))
        
        threats = []
        
        for i in range(start_idx, end_idx):
            if i < 0 or i >= len(msg.ranges): continue
            
            r = msg.ranges[i]
            if msg.range_min < r < detect_dist:
                # Calculate how "forward" this ray is (0.0 at center, 1.0 at far edge)
                angle_from_center = abs(i - center_idx) * msg.angle_increment
                angle_ratio = min(1.0, angle_from_center / zone_rad)
                
                # Apply Weight: 1.0 at center, sliding down to side_sensitivity at edge
                weight = 1.0 - (angle_ratio * (1.0 - side_weight))
                
                # Calculate threat for this specific ray (closer = higher)
                ray_threat = (detect_dist - r) / detect_dist
                threats.append(ray_threat * weight)
            else:
                # No obstacle seen by this ray
                threats.append(0.0)

        return sum(threats) / len(threats) if threats else 0.0

    def scan_callback(self, msg):
        zone_rad = np.deg2rad(self.p('zone_angle_deg'))
        center_idx = int(-msg.angle_min / msg.angle_increment)
        offset_idx = int(zone_rad / msg.angle_increment)
        
        # Calculate scores
        right_score = self.get_weighted_score(msg, center_idx - offset_idx, center_idx, center_idx)
        left_score = self.get_weighted_score(msg, center_idx, center_idx + offset_idx, center_idx)

        # STEERING: Left obstacles create negative steer (push right), Right create positive (push left)
        target_steer = (right_score - left_score) * self.p('steer_aggression')

        # SPEED: Based on the highest average threat seen in either zone
        max_threat = max(left_score, right_score)
        speed_factor = max(0.0, 1.0 - (max_threat * self.p('speed_aggression')))
        target_speed = self.p('min_speed') + (speed_factor * (self.p('max_speed') - self.p('min_speed')))

        # SMOOTHING
        alpha = self.p('smoothing_alpha')
        self.current_steer = (alpha * target_steer) + ((1.0 - alpha) * self.current_steer)
        self.current_speed = (alpha * target_speed) + ((1.0 - alpha) * self.current_speed)

        # Publish
        adj_msg = Float32MultiArray()
        adj_msg.data = [float(self.current_steer), float(self.current_speed)]
        self.adj_pub.publish(adj_msg)

def main():
    rclpy.init()
    rclpy.spin(ObstacleAvoidance())
    rclpy.shutdown()

if __name__ == '__main__': 
    main()