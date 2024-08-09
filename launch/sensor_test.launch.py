import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    
    # Setup project paths
    pkg_simulation2024 = get_package_share_directory('simulation2024')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch the Gazebo World
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_simulation2024,
            'worlds',
            'empty_world.sdf'
        ])}.items(),
    )

    # Include the lidar
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation2024, 'launch', 'lidar_test.launch.py')),
        launch_arguments={
            'X' : '-1.0',
            'Y' : '1.0',
        }.items(),
    )

    # Include the visible light camera
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation2024, 'launch', 'camera_test.launch.py')),
        launch_arguments={
            'X' : '-1.0',
            'Y' : '-1.0',
        }.items(),
    )

    # Include the thermal camera
    thermal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation2024, 'launch', 'thermal_test.launch.py')),
        launch_arguments={
            'X' : '1.0',
            'Y' : '1.0',
        }.items(),
    )

    # Include some aruco markers
    marker0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation2024, 'launch', 'aruco_marker.launch.py')),
        launch_arguments={
            'X' : '1.0',
            'Y' : '-1.0',
        }.items(),
    )

    # Start the bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        lidar,
        camera,
        thermal,
        marker0,
        bridge
    ])