#!/usr/bin/env python3
import rclpy, os, csv, yaml
import numpy as np
from rclpy.node import Node
from scipy.interpolate import splprep, splev
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, String, Bool, Int8

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        
        # Declare all parameters
        self.declare_parameters(namespace='', parameters=[
            ('waypoint_file', 'race3_wp3.csv'), 
            ('spacing', 0.10), ('spline_smooth', 1),
            ('max_v', 6), ('min_v', 0.9), 
            ('max_lat_accel', 5.0), ('max_lon_accel', 2.3), 
            ('lookahead_min', 1), ('lookahead_max', 2.7), ('lookahead_gain', 1.9), 
            ('k_sensitivity', 1.7), ('k_lookahead_window', 17),
            ('turn_p', 1.4),
            ('boost_scale', 3.0), ('boost_duration', 1.5), ('boost_ramp_down', 0.5)
        ])
        # Variables for storing waypoints
        self.waypoints = [] # Array of [x, y, velocity, curvature]
        self.pts = [] # Raw waypoints from CSV
        
        # Variables for obstacle avoidance adjustments
        self.obs_steering_offset = 0.0
        self.obs_speed_limit = float('inf')
        
        # ==================================== ROS PUBS and SUBS ====================================
        # Pure Pursuit
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.pose_sub = self.create_subscription(Odometry, '/pf/pose/odom', self.odom_callback, 10)
        # Debug
        self.target_pub = self.create_publisher(PointStamped, '/debug/target_wp', 10)
        self.path_pub = self.create_publisher(Path, '/debug/waypoints_path', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/debug/pure_pursuit_stats', 10)
        # Obstacle Avoidance
        self.obs_sub = self.create_subscription(Float32MultiArray, '/obstacle_adjustments', self.obs_callback, 10)

        # Parameters for agressive launch sequence
        self.boost_start_time = None
        self.boost_active = False
        self.boost_armed = False
        self.r1_was_pressed = False
        self.boost_sub = self.create_subscription(Int8, '/launch_boost', self._boost_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)

        # Setup for recalculating waypoints position, speed, and lookahead when params change
        self.add_on_set_parameters_callback(self.param_callback)
        self.load_and_process()

        # Share waypoint spline path with the debug node every 2 seconds
        self.create_timer(2.0, self.publish_path) 

    def p(self, name):       
        return self.get_parameter(name).value

    def obs_callback(self, msg):
        """Update adjustments based on the obstacle avoidance node."""
        if len(msg.data) == 2:
            self.obs_steering_offset = msg.data[0]
            self.obs_speed_limit = msg.data[1]

    def _boost_callback(self, msg: Int8):
        self.boost_start_time = self.get_clock().now()

    def _joy_callback(self, msg: Joy):
        # Square (buttons[0]) arms the boost.
        if len(msg.buttons) > 0 and msg.buttons[0] == 1 and not self.boost_armed:
            self.boost_armed = True
            self.get_logger().info("BOOST ARMED -- press R1 to launch")

        # R1 (buttons[5]) rising edge fires the boost if armed, then disarms.
        r1_pressed = len(msg.buttons) > 5 and msg.buttons[5] == 1
        if r1_pressed and not self.r1_was_pressed and self.boost_armed:
            self.boost_armed = False
            self.boost_start_time = self.get_clock().now()
        self.r1_was_pressed = r1_pressed

    def load_and_process(self):
        """Pipeline: Load CSV -> Interpolate -> Spline -> Physics Profile."""
        # ================================ Get waypoint file ================================
        pkg_path = os.path.join(os.environ.get('AMENT_PREFIX_PATH').split(':')[0], '..', '..', 
                                'src', 'pure_pursuit', 'waypoints')
        file_path = os.path.join(pkg_path, self.p('waypoint_file'))
        
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return

        with open(file_path, 'r') as f:
            self.pts = np.array([list(map(float, r)) for r in list(csv.reader(f))[1:] if r])
        # ================================ Spline Generation ================================
        spacing = self.p('spacing')
        lin_pts = []
        for i in range(len(self.pts)):
            # Get vector and absolute distance from one waypoint to the next
            p0, p1 = self.pts[i], self.pts[(i+1) % len(self.pts)]
            vec, dist = p1 - p0, np.linalg.norm(p1 - p0)
            if dist < 0.01: continue # Ignore waypoints right ontop of one another
            # Add interpolated points from one waypoint to the next
            for j in range(max(int(dist/spacing), 1)): 
                lin_pts.append(p0 + vec * (j/max(int(dist/spacing), 1)))

        # Try generating a smooth spline through all interpolated waypoints
        try:
            tck, _ = splprep(np.array(lin_pts).T, s=self.p('spline_smooth'), per=True)
            smooth_pts = np.array(splev(np.linspace(0, 1, len(lin_pts)), tck)).T
        except: smooth_pts = np.array(lin_pts)

        # ======= Waypoint Generation: calculate curvature, speed, lookahead using physics based parameters =======
        n = len(smooth_pts)
        k, v = np.zeros(n), np.zeros(n)
        win = 3 

        for i in range(n):
            # Get index of the previous and next waypoint from this one
            idx_p = (i - win) % n
            idx_n = (i + win) % n
            
            p0, p1, p2 = smooth_pts[idx_p], smooth_pts[i], smooth_pts[idx_n]
            va, vb = p1 - p0, p2 - p1
            la, lb = np.linalg.norm(va), np.linalg.norm(vb)
            
            # Use waypoints around this one to calculate the curvature of the raceline at this waypoint
            k[i] = np.arccos(np.clip(np.dot(va,vb)/(la*lb + 1e-6), -1, 1)) / ((la+lb)/2 + 1e-6)
            # Use curvature lateral acceleration params to calculate the target speed at this waypoint
            v[i] = np.sqrt(self.p('max_lat_accel') / (k[i] + 1e-6))
        
        v = np.clip(v, self.p('min_v'), self.p('max_v'))
        
        # Forward/Backward passes to respect longitudinal acceleration limits
        alon = self.p('max_lon_accel')
        for _ in range(2):
            for i in reversed(range(n)):
                nxt = (i+1)%n
                ds = np.linalg.norm(smooth_pts[nxt]-smooth_pts[i])
                v[i] = min(v[i], np.sqrt(v[nxt]**2 + 2*alon*ds))
            for i in range(n):
                prv = (i-1)%n
                ds = np.linalg.norm(smooth_pts[i]-smooth_pts[prv])
                v[i] = min(v[i], np.sqrt(v[prv]**2 + 2*alon*ds))

        # Return newly calculated waypoints
        self.waypoints = np.column_stack((smooth_pts, v, k))

    def odom_callback(self, msg):
        if not len(self.waypoints): return

        # Find the index of the waypoint the car is closest to
        curr = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        idx = np.argmin(np.linalg.norm(self.waypoints[:,:2] - curr, axis=1))
        
        # ======== ADAPTIVE LOOKAHEAD: Shrink Ld if high curvature is detected ahead ========
        win = self.p('k_lookahead_window')
        ahead_k = np.max(self.waypoints[[(idx + j - 3) % len(self.waypoints) for j in range(win)], 3])
        
        Ld = (self.p('lookahead_gain') * self.waypoints[idx, 2]) / \
             (1.0 + (ahead_k * self.p('k_sensitivity')))
        Ld = np.clip(Ld, self.p('lookahead_min'), self.p('lookahead_max'))

        # ======== PURSUIT: Find target point and calculate steering ========
        target = self.waypoints[idx]
        for i in range(len(self.waypoints)):
            wp = self.waypoints[(idx + i) % len(self.waypoints)]
            if np.linalg.norm(wp[:2] - curr) > Ld:
                target = wp
                break

        # Transform target to local frame for lateral error (ly)
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        dx, dy = target[0]-curr[0], target[1]-curr[1]
        ly = -dx * np.sin(yaw) + dy * np.cos(yaw) 
        
        # ======================== Obstacle Avoidance ======================== 
        # Cap the speed if there is a wall/blockage
        base_speed = min(float(self.waypoints[idx][2]), self.obs_speed_limit)
        # Add the steering offset to avoid obstacles
        base_steering = (self.p('turn_p') * ly) / (Ld**2)
        final_steering = base_steering + self.obs_steering_offset

        # ======= Launch boost: scale waypoint speed (ignoring obs cap) and force straight ========
        final_speed = base_speed
        scale = self.p('boost_scale')
        if self.boost_start_time is not None and scale > 1.0:
            elapsed = (self.get_clock().now() - self.boost_start_time).nanoseconds * 1e-9
            dur = self.p('boost_duration')
            ramp = self.p('boost_ramp_down')
            if elapsed < dur:
                if not self.boost_active:
                    self.boost_active = True
                    self.get_logger().info(
                        f"BOOST ENGAGED: scale={scale}, dur={dur}s, ramp={ramp}s, "
                        f"wp_speed={float(self.waypoints[idx][2]):.2f} -> {float(self.waypoints[idx][2]) * scale:.2f} m/s"
                    )
                final_speed = float(self.waypoints[idx][2]) * scale
                final_steering = 0.0
            elif elapsed < dur + ramp and ramp > 0:
                # Blend speed back and let PP steering re-engage so it can correct any drift accumulated during the straight
                if self.boost_active:
                    self.boost_active = False
                    self.get_logger().info(f"BOOST RAMP-DOWN at t={elapsed:.2f}s")
                alpha = (elapsed - dur) / ramp
                boosted = float(self.waypoints[idx][2]) * scale
                final_speed = (1.0 - alpha) * boosted + alpha * base_speed
            else:
                if self.boost_active:
                    self.boost_active = False
                self.get_logger().info(f"BOOST DONE at t={elapsed:.2f}s")
                self.boost_start_time = None

        # Command Drive
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.speed = final_speed
        drive.drive.steering_angle = np.clip(float(final_steering), -0.4, 0.4)
        self.drive_pub.publish(drive)

        # Debug Visuals
        tp = PointStamped()
        tp.header.frame_id, tp.header.stamp = "map", self.get_clock().now().to_msg()
        tp.point.x, tp.point.y = target[0], target[1]
        self.target_pub.publish(tp)

        # Create debug message
        debug_msg = Float32MultiArray()
        debug_msg.data = [float(Ld), float(target[2]), float(ahead_k), float(ly)]
        self.debug_pub.publish(debug_msg)

    def param_callback(self, params):
        # Deferred execution via one-shot timer ensures params are fully saved in ROS map
        self.update_timer = self.create_timer(0.1, self.delayed_load)
        return SetParametersResult(successful=True)

    def delayed_load(self):
        self.update_timer.cancel()
        self.destroy_timer(self.update_timer)
        self.load_and_process()

    def publish_path(self):
        if not len(self.waypoints): return
        msg = Path()
        msg.header.frame_id, msg.header.stamp = "map", self.get_clock().now().to_msg()
        for wp in self.waypoints:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = wp[0], wp[1]
            msg.poses.append(p)
        self.path_pub.publish(msg)

def main():
    rclpy.init(); rclpy.spin(PurePursuit()); rclpy.shutdown()

if __name__ == '__main__': main()