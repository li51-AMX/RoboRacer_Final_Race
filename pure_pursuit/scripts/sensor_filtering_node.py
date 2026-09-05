#!/usr/bin/env python3
# ROS Imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan

# Other Imports
import numpy as np
from collections import deque

class LidarFilterNode(Node):
    """ 
    Subscribes to raw LiDAR scans, applies filtering (mean window, NaN rejection, 
    max distance clipping), and publishes a filtered LaserScan message.
    """
    def __init__(self):
        super().__init__('lidar_filter_node')
        
        # Topics
        raw_lidar_topic = '/scan'
        filtered_lidar_topic = '/scan_filtered'

        # Create subscriber to raw LiDAR scans
        self.scan_sub = self.create_subscription(LaserScan, raw_lidar_topic, self.lidar_callback, 10)

        # Create publisher for the filtered LiDAR scans
        self.scan_pub = self.create_publisher(LaserScan, filtered_lidar_topic, 10)

        # Declare parameters for tuning the filter
        self.declare_parameter('max_lidar', 3.0)
        self.declare_parameter('window_size', 10)

        # Fetch parameter values
        self.max_lidar = self.get_parameter('max_lidar').value
        self.window_size = self.get_parameter('window_size').value

        # Mean filter window for scan preprocess
        self.ranges_history = deque(maxlen=self.window_size)

        # Parameter callback handle
        self.param_cb_handle = self.add_on_set_parameters_callback(self._on_param_change)

    def _on_param_change(self, params):
        """ Allow dynamic tuning of the filter parameters """
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            if param.name == 'max_lidar':
                self.max_lidar = param.value
            elif param.name == 'window_size':
                # If window size changes, we need to rebuild the deque
                self.window_size = param.value
                current_data = list(self.ranges_history)
                self.ranges_history = deque(current_data, maxlen=self.window_size)
                
        return result

    def lidar_callback(self, msg):
        """ 
        Process each raw LiDAR scan, filter it, and publish the cleaned message.
        """
        ranges = np.array(msg.ranges)
        max_dist = self.max_lidar

        # Create array to hold processed ranges
        proc_ranges = np.zeros_like(ranges)
        
        # Handle invalid measurements (NaNs, Inf, and out of bounds)
        for i, range_val in enumerate(ranges):
            if np.isnan(range_val) or np.isinf(range_val) or range_val > max_dist:
                proc_ranges[i] = max_dist
            else:
                # Ensure non-zero value
                proc_ranges[i] = max(1e-9, range_val)

        # Apply rolling mean filter
        self.ranges_history.append(proc_ranges)
        filtered_ranges = np.mean(self.ranges_history, axis=0)

        # Construct the new filtered LaserScan message
        filtered_msg = LaserScan()
        
        # Copy header and metadata from the original message
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = float(self.max_lidar)
        
        # Convert the filtered numpy array back to a standard Python list
        filtered_msg.ranges = filtered_ranges.tolist()
        
        # Pass through intensities if the original message had them
        if len(msg.intensities) > 0:
            filtered_msg.intensities = msg.intensities

        # Publish the filtered scan
        self.scan_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    print("LiDAR Filter Node Initialized")
    filter_node = LidarFilterNode()
    
    try:
        rclpy.spin(filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        filter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()