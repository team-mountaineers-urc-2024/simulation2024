# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_simulation2024 = get_package_share_directory('simulation2024')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_simulation2024,
            'worlds',
            'world2.sdf'
        ])}.items(),
    )

    wanderer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # os.path.join(pkg_simulation2024, 'launch', 'wanderer_II.launch.py')),
            os.path.join(pkg_simulation2024, 'launch', 'heimdall.launch.py')),
        launch_arguments={
            'X' : '100.0',
            'Y' : '-85.0',
        }.items(),
    )

    # Start the bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # arguments=['/front_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        #            '/front_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        #            '/back_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        #            '/back_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        #            '/left_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        #            '/left_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        #            '/right_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        #            '/right_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        #            '/fumunda_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        #            '/fumunda_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        #            'lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        #            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        #            '/model/wanderer_II/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        #            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        arguments=['/fir_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/fir_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/fil_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/fil_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/bir_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/bir_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/bil_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/bil_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/frontr_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/frontr_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/frontl_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/frontl_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/model/wanderer_II/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        wanderer
    ])