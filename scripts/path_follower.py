#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, TwistStamped
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Create variables 
        self.path = None
        self.current_pose = None
        self.waypoint_idx = 0

        self.create_subscription(Path, '/gpmp2_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Path Follower Ready")
    
    # Callback functions
    def path_callback(self, msg):
        self.path = msg
        self.waypoint_idx = 0
        self.get_logger().info(f"Received path with {len(msg.poses)} poses")
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    # Main controller loop
    def control_loop(self):
        if self.path is None or self.current_pose is None:
            return
        
        if self.waypoint_idx >= len(self.path.poses):
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            return
        
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.get_yaw(self.current_pose.orientation)
        
        target = self.path.poses[self.waypoint_idx].pose.position
        
        dx = target.x - x
        dy = target.y - y
        dist = math.sqrt(dx*dx + dy*dy)
        angle_to_target = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_target - yaw)
        
        if dist < 0.15:
            self.waypoint_idx += 1
            return
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        if abs(angle_error) > 0.3:
            cmd.twist.angular.z = 0.5 * angle_error / abs(angle_error)
        else:
            cmd.twist.linear.x = 0.15
            cmd.twist.angular.z = 0.5 * angle_error
        
        self.cmd_pub.publish(cmd)
    
    def get_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))

    def normalize_angle(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi 
        return a
    
def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()