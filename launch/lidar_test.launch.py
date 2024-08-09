import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
import launch_ros.descriptions as descriptions
import launch.actions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():

    # Set up the default paths for things
    pkg_share = FindPackageShare(package='simulation2024').find('simulation2024')
    default_model_path = os.path.join(pkg_share, 'models', 'sensors', 'lidar_test.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config' 'urdf_config.rviz')

    # Get the urdf file from xacro
    # TODO, allow arguments to change path
    state_publisher_config = xacro.process_file(
        default_model_path,
        mappings={

        },
    )

    # Launch the state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='lidar_test',
        parameters=[
            # Use xacro to correctly spawn the correct lidar
            {'robot_description': state_publisher_config.toxml()
            }]
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='lidar_test',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    # Spawn the lidar
    lidar_test = Node(
        package='ros_gz_sim', 
        executable='create',
        namespace='lidar_test',
        arguments=['-entity', 'lidar_test', 
                   '-name', 'lidar_test',
                    '-topic', 'robot_description',
                        '-x', LaunchConfiguration('X'),
                        '-y', LaunchConfiguration('Y'),
                        '-z', '0.0',
                        '-Y', '0.0'],
                        output='screen')

    # Link the lidar frame to the tree
    tf_link = Node(
        package = "tf2_ros",
        executable="static_transform_publisher",
        arguments = ['--x','0',
                        '--y','0',
                        '--z','0',
                        '--roll','0',
                        '--pitch','0',
                        '--yaw','0',
                        '--frame-id','lidar',
                        '--child-frame-id','lidar_test/base_link/lidar'],
        )

    ld = launch.LaunchDescription([

        # Is rViz wanted
        launch.actions.DeclareLaunchArgument(name='rviz', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),

        # What is the model file path
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to mallet urdf file'),

        # What is the rviz config file path
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        # What is the lidar X coordinate
        launch.actions.DeclareLaunchArgument(name='X', default_value='0.0',
                                            description='The X position for the mallet to be spawned'),

        # What is the lidar Y coordinate
        launch.actions.DeclareLaunchArgument(name='Y', default_value='0.0',
                                            description='The Y position for the mallet to be spawned')
        ])
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(lidar_test)
    ld.add_action(tf_link)

    return ld