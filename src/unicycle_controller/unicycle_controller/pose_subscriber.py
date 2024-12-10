#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import math

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(TFMessage, "/tf", self.pose_callback, 10 )

    def pose_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                # Extract translation
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                
                # Extract rotation (quaternion)
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                
                # Convert quaternion to yaw (Euler angle)
                yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                
                # Log the position and yaw
                self.get_logger().info(f"Position: ({x}, {y}), Yaw: {yaw:.4f} radians")
                

def main(args = None):
    rclpy.init(args=args)  # Initialize ROS 2 Python
    print("Starting the PoseSubscriberNode...")
    try:
        node = PoseSubscriberNode()
        rclpy.spin(node)  # Keep the node alive and responsive to callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shutdown ROS 2
