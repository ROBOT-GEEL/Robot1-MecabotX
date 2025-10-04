#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
import math

class DriftFinder(Node):
    def __init__(self):
        super().__init__('drift_finder')

        self.amcl_pose = None
        self.odom_pose = None

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/odom_combined', self.odom_callback, 10)

        self.get_logger().info("Drift Finder started. Listening to /amcl_pose and /odom_combined...")

    def amcl_callback(self, msg):
        self.amcl_pose = msg
        self.check_drift()

    def odom_callback(self, msg):
        self.odom_pose = msg
        self.check_drift()

    def check_drift(self):
        if self.amcl_pose is None or self.odom_pose is None:
            return
        
        amcl_yaw = self.get_yaw(self.amcl_pose.pose.pose.orientation)
        odom_yaw = self.get_yaw(self.odom_pose.pose.pose.orientation)

        drift = self.angle_diff(amcl_yaw, odom_yaw)

        self.get_logger().info(f"Yaw Drift: {math.degrees(drift):.2f}° (AMCL: {math.degrees(amcl_yaw):.2f}°, Odom: {math.degrees(odom_yaw):.2f}°)")

    def get_yaw(self, orientation_q):
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

def main(args=None):
    rclpy.init(args=args)
    node = DriftFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

