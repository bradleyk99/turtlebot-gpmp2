#!/usr/bin/env python3
# Node for GPMP2 Path Planner


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from gpmp2_planner.srv import PlanPath

class GPMP2Planner(Node):
    def __init__(self):
        super().__init__('gpmp2_planner')

        # Data storage
        self.map_data = None
        self.robot_pose = None
        self.goal_pose = None

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)

        # Path publisher
        self.path_pub = self.create_publisher(Path, '/gpmp2_path', 10)

        self.plan_client = self.create_client(PlanPath, 'plan_path')

        self.get_logger().info("GPMP2 Planner Node Ready")

    def map_callback(self, msg):
        self.map_data = msg
    
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info("Received a Goal")
        self.plan_path()
    
    def plan_path(self):
        # Check that all required inputs are available
        if self.map_data is None:
            self.get_logger().warn("Cannot plan: no map received yet.")
            return
        if self.robot_pose is None:
            self.get_logger().warn("Cannot plan: robot pose not received yet.")
            return
        if self.goal_pose is None:
            self.get_logger().warn("Cannot plan: goal not received yet.")
            return
        
        # Check service availability
        if not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("GPMP2 planning service is not available")
            return
        
        # Build request
        request = PlanPath.Request()
        request.map = self.map_data
        request.start = self.robot_pose
        request.goal = self.goal_pose

        self.get_logger().info("Calling GPMP2 planning service...")

        # Async call
        future = self.plan_client.call_async(request)
        future.add_done_callback(self.plan_response_callback)
    
    def plan_response_callback(self, future):
        try: 
            response = future.result()
            if response.success:
                self.get_logger().info(f"Planning succeeded! Path has {len(response.path.poses)} poses")
                self.path_pub.publish(response.path)
            else:
                self.get_logger().error(f"Planning Failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
def main(args=None):
    rclpy.init(args=args)
    node = GPMP2Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()