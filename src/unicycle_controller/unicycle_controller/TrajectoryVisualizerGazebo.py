import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
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

class TrajectoryVisualizerGazebo(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer_gazebo')
        self.publisher = self.create_publisher(Marker, 'trajectory_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)

        self.scaling_factor = 2

        path = "/home/daniele/ros2_ws/src/unicycle_controller/unicycle_controller/trajectory.txt"
        #path = "trajectory.txt"
        global_trajectory = create_trajectory(path)

        self.trajectory = global_trajectory / self.scaling_factor

        self.spawn_service = self.create_client(SpawnEntity, '/spawn_entity')

        # Wait for the service to be available
        self.spawn_service.wait_for_service()

        # Spawn each point
        for i, point in enumerate(self.trajectory):
            self.spawn_model(point, f"trajectory_point_{i}",self.scaling_factor)

    
    def publish_marker(self):
        # Trajectory points (your matrix)
        

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
        for point in self.trajectory:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1]-22/self.scaling_factor + 1)
            p.z = float(0)  # Flat trajectory
            marker.points.append(p)

        self.publisher.publish(marker)
        self.get_logger().info('Published trajectory marker')

        

    def spawn_model(self, point, model_name, scaling_factor):
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = """
        <sdf version="1.6">
            <model name="{name}">
                <static>true</static>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                            <sphere>
                                <radius>0.02</radius>
                            </sphere>
                        </geometry>
                        <material>
                            <ambient>1.0 0.0 0.0 1.0</ambient>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        """.format(name=model_name)

        # Set position from the trajectory point
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(point[0])
        req.initial_pose.position.y = float(point[1]-22/ scaling_factor + 1)
        req.initial_pose.position.z = 0.0

        future = self.spawn_service.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f"Spawned {model_name} at ({point[0]}, {point[1]})")
        else:
            self.get_logger().error(f"Failed to spawn {model_name}: {future.result().status_message}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizerGazebo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


