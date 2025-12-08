#!/usr/bin/env python3
# Node for GPMP2 Path Planner
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from gpmp2_planner.srv import PlanPath
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
import yaml
import numpy as np
from PIL import Image
import os


class GPMP2Planner(Node):
    def __init__(self):
        super().__init__('gpmp2_planner')
        
        # Data storage
        self.map_data = None
        self.robot_pose = None
        self.goal_pose = None
        
        # Load map directly from file
        self.load_map_from_file()
        
        # Publisher for RViz visualization (with transient local so RViz gets it)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        
        # Publish map periodically so RViz can see it
        self.create_timer(1.0, self.publish_map)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Path publisher
        self.path_pub = self.create_publisher(Path, '/gpmp2_path', 10)
        
        # Service client
        self.plan_client = self.create_client(PlanPath, 'plan_path')
        
        self.get_logger().info("GPMP2 Planner Node Ready")

    def load_map_from_file(self):
        """Load map directly from YAML + image file"""
        try:
            package_dir = get_package_share_directory('gpmp2_planner')
            yaml_path = os.path.join(package_dir, 'maps', 'my_map.yaml')
            
            self.get_logger().info(f"Loading map from: {yaml_path}")
            
            with open(yaml_path, 'r') as f:
                map_info = yaml.safe_load(f)
            
            # Load the image (path is relative to yaml file)
            image_path = os.path.join(os.path.dirname(yaml_path), map_info['image'])
            img = Image.open(image_path)
            map_array = np.array(img)
            
            # Convert to occupancy grid format
            # PGM values: 255=free, 0=occupied, 205=unknown (typical)
            occupancy = np.zeros_like(map_array, dtype=np.int8)
            
            free_thresh = map_info.get('free_thresh', 0.196) * 255
            occupied_thresh = map_info.get('occupied_thresh', 0.65) * 255
            
            occupancy[map_array > (255 - free_thresh)] = 0       # Free
            occupancy[map_array < occupied_thresh] = 100          # Occupied  
            occupancy[(map_array >= occupied_thresh) & (map_array <= (255 - free_thresh))] = -1  # Unknown
            
            # Flip vertically (image origin is top-left, map origin is bottom-left)
            occupancy = np.flipud(occupancy)
            
            # Build OccupancyGrid message
            self.map_data = OccupancyGrid()
            self.map_data.header.frame_id = 'map'
            self.map_data.header.stamp.sec = 0
            self.map_data.header.stamp.nanosec = 0
            self.map_data.info.resolution = float(map_info['resolution'])
            self.map_data.info.width = map_array.shape[1]
            self.map_data.info.height = map_array.shape[0]
            self.map_data.info.origin.position.x = float(map_info['origin'][0])
            self.map_data.info.origin.position.y = float(map_info['origin'][1])
            self.map_data.info.origin.position.z = 0.0
            self.map_data.data = occupancy.flatten().tolist()
            
            self.get_logger().info(
                f"Map loaded: {self.map_data.info.width}x{self.map_data.info.height} "
                f"@ {self.map_data.info.resolution}m/cell"
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            self.map_data = None

    def publish_map(self):
        """Publish map for RViz visualization"""
        if self.map_data is not None:
            self.map_pub.publish(self.map_data)

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info("Received a Goal")
        self.plan_path()

    def plan_path(self):
        if self.map_data is None:
            self.get_logger().warn("Cannot plan: no map loaded.")
            return
        if self.robot_pose is None:
            self.get_logger().warn("Cannot plan: robot pose not received yet.")
            return
        if self.goal_pose is None:
            self.get_logger().warn("Cannot plan: goal not received yet.")
            return

        if not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("GPMP2 planning service is not available")
            return

        request = PlanPath.Request()
        request.map = self.map_data
        request.start = self.robot_pose
        request.goal = self.goal_pose

        self.get_logger().info("Calling GPMP2 planning service...")
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