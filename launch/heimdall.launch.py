import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.descriptions as descriptions
import launch.actions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():

    # Set up the default paths for things
    pkg_share = FindPackageShare(package='simulation2024').find('simulation2024')
    default_model_path = os.path.join(pkg_share, 'models', 'heimdall', 'heimdall.xacro')
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
        namespace='heimdall',
        parameters=[
            # Use xacro to correctly spawn the correct lidar
            {'robot_description': state_publisher_config.toxml()
            }]
    )

    # Launch the joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='heimdall',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    # Launch the joint publisher gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='heimdall',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='heimdall',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    # Spawn the wanderer model
    rover_spawn = Node(
        package='ros_gz_sim', 
        executable='create',
        namespace='heimdall',
        arguments=['-entity', 'heimdall', 
                   '-name', 'heimdall',
                    '-topic', 'robot_description',
                        '-x', LaunchConfiguration('X'),
                        '-y', LaunchConfiguration('Y'),
                        '-z', '15',
                        '-Y', '2.5'],
                        output='screen')

    ld = launch.LaunchDescription([
        # Is the Joint State GUI wanted
        launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),

        # Is rViz wanted
        launch.actions.DeclareLaunchArgument(name='rviz', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),

        # What is the model file path
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),

        # What is the rviz config file path
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        # What is the marker X coordinate
        launch.actions.DeclareLaunchArgument(name='X', default_value='0.0',
                                            description='The X position for the marker to be spawned'),

        # What is the marker Y coordinate
        launch.actions.DeclareLaunchArgument(name='Y', default_value='0.0',
                                            description='The Y position for the marker to be spawned')
        ])
    
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(rover_spawn)

    return ld