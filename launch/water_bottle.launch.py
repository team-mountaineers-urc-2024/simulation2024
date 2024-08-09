import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.descriptions as descriptions
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():

    # Set up the default paths for things
    pkg_share = FindPackageShare(package='simulation2024').find('simulation2024')
    default_model_path = os.path.join(pkg_share, 'models', 'water_bottle', 'water_bottle.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config' 'urdf_config.rviz')

    # Launch the state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='water_bottle',
        parameters=[
            # Use xacro to correctly spawn the correct mallet
            {'robot_description': descriptions.ParameterValue(
                Command(['xacro ', LaunchConfiguration('model'),
                         ' red:=', LaunchConfiguration('red'),
                         ' green:=', LaunchConfiguration('green'),
                         ' blue:=', LaunchConfiguration('blue'),
                         ' alpha:=', LaunchConfiguration('alpha')
                         ]),
                value_type=str)
            }]
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='water_bottle',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    # Spawn the water bottle model
    aruco_spawn = Node(
        package='ros_gz_sim', 
        executable='create',
        namespace='water_bottle',
        arguments=['-entity', 'water_bottle', 
                   '-name', 'water_bottle',
                    '-topic', 'robot_description',
                        '-x', LaunchConfiguration('X'),
                        '-y', LaunchConfiguration('Y'),
                        '-z', '13.9',
                        '-Y', '0.0'],
                        output='screen')

    ld = launch.LaunchDescription([
        
        # Is rViz wanted
        launch.actions.DeclareLaunchArgument(name='rviz', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),

        # What is the model file path
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to bottle urdf file'),

        # What is the rviz config file path
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        # What is the bottle X coordinate
        launch.actions.DeclareLaunchArgument(name='X', default_value='0.0',
                                            description='The X position for the bottle to be spawned'),

        # What is the bottle Y coordinate
        launch.actions.DeclareLaunchArgument(name='Y', default_value='0.0',
                                            description='The Y position for the bottle to be spawned'),

        # What is the red value of the bottle
        launch.actions.DeclareLaunchArgument(name='red', default_value='0.0',
                                            description='The red value for the water bottle color'),

        # What is the green value of the bottle
        launch.actions.DeclareLaunchArgument(name='green', default_value='0.0',
                                            description='The green value for the water bottle color'),

        # What is the blue value of the bottle
        launch.actions.DeclareLaunchArgument(name='blue', default_value='0.0',
                                            description='The blue value for the water bottle color'),

        # What is the alpha value of the bottle
        launch.actions.DeclareLaunchArgument(name='alpha', default_value='1.0',
                                            description='The alpha value for the water bottle color')
        ])
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(aruco_spawn)

    return ld