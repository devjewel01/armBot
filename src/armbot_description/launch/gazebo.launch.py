import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    armbot_description_dir = get_package_share_directory('armbot_description')
    armbot_description_share = os.path.join(get_package_prefix('armbot_description'), 'share')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Argument for the model file path
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(armbot_description_dir, 'urdf', 'armbot.urdf.xacro'),
        description='Absolute path to robot URDF file'
    )

    # Set the GAZEBO_MODEL_PATH environment variable
    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', armbot_description_share)

    # Robot description parameter
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Start Gazebo Sim server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4']}.items()  # Run with verbose output
    )

    # Spawn robot in Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'armbot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        robot_state_publisher_node,
        spawn_robot
    ])
