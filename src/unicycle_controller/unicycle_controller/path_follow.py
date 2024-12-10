#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker 
from scipy.interpolate import interp1d
import math
import numpy as np

def interpolate_path(points, resolution=0.01):
    points = np.array(points)
    if len(points) < 2:
        return points  # Cannot interpolate a single point or empty path
    
    # Extract x and y coordinates
    x = points[:, 0]
    y = points[:, 1]

    # Calculate the distance between waypoints
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    cumulative_distances = np.hstack(([0], np.cumsum(distances)))

    # Generate evenly spaced distances for interpolation
    new_distances = np.arange(0, cumulative_distances[-1], resolution)

    # Interpolate x and y coordinates based on the new distances
    interp_x = interp1d(cumulative_distances, x, kind='linear')
    interp_y = interp1d(cumulative_distances, y, kind='linear')

    # Generate interpolated waypoints
    new_x = interp_x(new_distances)
    new_y = interp_y(new_distances)

    return list(zip(new_x, new_y))

class PathFollow(Node):

    def __init__(self):
        super().__init__("path_follow")

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/cmd_vel",10)
        self.pose_subscriber_ = self.create_subscription(TFMessage, "/tf", self.pose_callback, 10)
        self.marker_subscriber_ = self.create_subscription(Marker, "/trajectory_marker", self.marker_callback, 10)

        self.get_logger().info("Path control has been started")

        # Timer for control loop
        self.control_timer_ = self.create_timer(0.1, self.control_callback)  # 10 Hz control frequency

        # Internal state
        self.current_pose_ = None  # (x, y, yaw)
        self.path_ = []  # List of waypoints [(x1, y1), (x2, y2), ...]
        self.lookahead_steps = 3 # Number of steps ahead for lookahead point

        self.get_logger().info("Path control has been started.")

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
                
                self.current_pose_ = (x, y, yaw)
                self.get_logger().info(f"Updated Pose: ({x:.2f}, {y:.2f}), Yaw: {yaw:.4f} radians")


    def marker_callback(self, msg: Marker):
        # Callback for receiving the trajectory from Marker

        # Extract the waypoints from the marker
        #raw_path = [(point.x, point.y) for point in msg.points]
        
        # Interpolate the path to make it continuous
        #self.path_ = interpolate_path(raw_path, resolution=0.1)
        
        self.path_ = [(point.x, point.y) for point in msg.points]
        self.get_logger().info(f"Received trajectory with {len(self.path_)} points.")

    
    def control_callback(self):

        if self.current_pose_ is None or not self.path_:
            self.get_logger().warn("Pose or path not available, skipping control.")
            return
        
        # Extract the robot's current pose
        x, y, yaw = self.current_pose_
        sign_angle = 1

        # Find the closest waypoint in the path
        closest_point = min(self.path_, key=lambda wp: math.hypot(wp[0] - x, wp[1] - y))
        closest_index = self.path_.index(closest_point)

        # Determine the lookahead point index
        lookahead_index = closest_index + self.lookahead_steps
        if lookahead_index >= len(self.path_):
            lookahead_index = len(self.path_) - 1  # Clamp to the last point

        # Calculate control commands (simple proportional controller example)
        # Set the lookahead point
        lookahead_point = self.path_[lookahead_index]
        goal_x, goal_y = lookahead_point

        distance = math.hypot(goal_x - x, goal_y - y)
        angle_to_goal = math.atan2(goal_y - y, goal_x - x)
        angle_error = angle_to_goal - yaw

        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Define the sign of the angle
        if angle_error >= 0:
            sign_angle = 1
        elif angle_error <0:
            sign_angle = -1

        # Proportional control gains
        k_linear = 0.6
        k_angular = 0.8

        if angle_error <= 0.1:
            k_e = 0
        else:
            k_e = 0.6

        # Compute control commands
        linear_velocity = k_linear * distance
        angular_velocity = k_angular * (angle_error + sign_angle * math.atan(k_e * distance / linear_velocity))

        # Publish control commands
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.cmd_vel_publisher_.publish(cmd)

        self.get_logger().info(
            f"Control: Linear={linear_velocity:.2f}, Angular={angular_velocity:.2f}")


def main(args = None):
    rclpy.init(args = args)
    node = PathFollow()
    rclpy.spin(node)
    rclpy.shutdown()