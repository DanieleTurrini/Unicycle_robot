import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

def create_trajectory(file_path):
    """
    Reads a text file with a specific format and converts it into a numpy array.

    Args:
        file_path (str): Path to the input text file.

    Returns:
        np.ndarray: A numpy array with the parsed data.
    """
    data = []
    
    # Open the file and read line by line
    with open(file_path, 'r') as file:
        for line in file:
            # Split the line into two numbers
            parts = line.strip().split()
            if len(parts) == 2:
                # Convert strings to floats and append to the list
                data.append([float(parts[0]), float(parts[1])])
    
    # Convert the list to a numpy array
    trajectory = np.array(data)
    return trajectory

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        self.publisher = self.create_publisher(Marker, 'trajectory_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)

    def publish_marker(self):
        # Trajectory points (your matrix)
        scaling_factor = 4
        path = "/home/daniele/ros2_ws/src/unicycle_controller/unicycle_controller/trajectory.txt"
        #path = "trajectory.txt"
        global_trajectory = create_trajectory(path)

        trajectory = global_trajectory / scaling_factor

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP  # or Marker.SPHERE_LIST for individual points
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        # Add trajectory points to the marker
        for point in trajectory:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1]-22/scaling_factor + 1)
            p.z = float(0)  # Flat trajectory
            marker.points.append(p)

        self.publisher.publish(marker)
        self.get_logger().info('Published trajectory marker')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
