from setuptools import find_packages, setup

package_name = 'unicycle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniele',
    maintainer_email='daniele@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pose_subscriber = unicycle_controller.pose_subscriber:main",
            "TrajectoryVisualizer = unicycle_controller.TrajectoryVisualizer:main",
            "path_follow = unicycle_controller.path_follow:main",
            "TrajectoryVisualizerGazebo = unicycle_controller.TrajectoryVisualizerGazebo:main"
        ],
    },
)
