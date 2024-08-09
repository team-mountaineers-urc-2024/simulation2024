import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.descriptions as descriptions
import launch.actions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Set up the default paths for things
    pkg_share = FindPackageShare(package='simulation2024').find('simulation2024')
    default_model_path = os.path.join(pkg_share, 'models', 'deadblow_mallet', 'mallet.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config' 'urdf_config.rviz')

    # Launch the state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='deadblow_mallet',
        parameters=[
            # Use xacro to correctly spawn the correct mallet
            {'robot_description': descriptions.ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str)
            }]
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='deadblow_mallet',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    # Spawn the aruco model
    aruco_spawn = Node(
        package='ros_gz_sim', 
        executable='create',
        namespace='deadblow_mallet',
        arguments=['-entity', 'deadblow_mallet', 
                   '-name', 'deadblow_mallet',
                    '-topic', 'robot_description',
                        '-x', LaunchConfiguration('X'),
                        '-y', LaunchConfiguration('Y'),
                        '-z', '15',
                        '-Y', '0.0'],
                        output='screen')

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

        # What is the mallet X coordinate
        launch.actions.DeclareLaunchArgument(name='X', default_value='0.0',
                                            description='The X position for the mallet to be spawned'),

        # What is the mallet Y coordinate
        launch.actions.DeclareLaunchArgument(name='Y', default_value='0.0',
                                            description='The Y position for the mallet to be spawned')
        ])
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(aruco_spawn)

    return ld