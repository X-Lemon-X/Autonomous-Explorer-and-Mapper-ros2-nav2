import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")
        self.timer = self.create_timer(1.0, self.callback_timer)

    def callback_timer(self):
      x,y=self.get_robot_pose()
      self.get_logger().info(f"Current POSE: x={x} , y={y}")

    def get_robot_pose(self, target_frame='map', source_frame='base_link', timeout=5.0):
      """
      Get the robot's current pose from TF2.

      Args:
          node (rclpy.node.Node): A running ROS 2 node.
          target_frame (str): The global frame (usually 'map' or 'odom').
          source_frame (str): The robot base frame (usually 'base_link').
          timeout (float): Timeout for transform lookup in seconds.

      Returns:
          tuple: (x, y, theta) in meters and radians.
      """
      tf_buffer = tf2_ros.Buffer()
      tf_listener = tf2_ros.TransformListener(tf_buffer, self)

      try:
          # Wait for the transform to become available
          pose_cam = PoseStamped()
          pose_cam.header.stamp = self.get_clock().now().to_msg()
          pose_cam.header.frame_id = source_frame
          pose_cam.pose.position.x = 0.0
          pose_cam.pose.position.y = 0.0
          pose_cam.pose.position.z = 0.0
          pose_cam.pose.orientation.w = 1.0
          self.get_logger().info(f"Looking up transform from {target_frame} to {source_frame}")
          transform: TransformStamped = tf_buffer.lookup_transform(
              target_frame,
              source_frame,
              now,
              rclpy.duration.Duration(seconds=timeout)
          )

          # Extract translation
          x = transform.transform.translation.x
          y = transform.transform.translation.y
          return x, y

      except TransformException as ex:
          self.get_logger().warn(f'Could not transform {source_frame} to {target_frame}: {ex}')
          return None,None
      
def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()
