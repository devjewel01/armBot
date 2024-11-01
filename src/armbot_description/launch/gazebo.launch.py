import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Directories
    armbot_description_dir = get_package_share_directory('armbot_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Argument for the model file path
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(armbot_description_dir, 'urdf', 'armbot.urdf.xacro'),
        description='Absolute path to robot URDF file'
    )

    # Set the GAZEBO_MODEL_PATH environment variable
    env_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(armbot_description_dir)
    )

    # Robot description parameter
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )

    # Default world file for Gazebo Sim
    default_world = os.path.join(
        armbot_description_dir,
        'worlds',
        'empty.world'
    )

    # Declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Launch Gazebo Sim server with a specified world
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4 ', LaunchConfiguration('world')]}.items()  # Run with verbose logging
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn robot in Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'armbot', '-topic', 'robot_description'],
        output='screen'
    )

    # Bridge parameters for custom topic bridging
    bridge_params = os.path.join(
        get_package_share_directory('armbot_controller'), 'config', 'gz_bridge.yaml'
    )

    # ROS-GZ bridge for custom parameter bridging
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    # Image bridge for camera topic
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    # Return the complete launch description
    return LaunchDescription([
        env_var,
        model_arg,
        world_arg,
        start_gazebo_server,
        robot_state_publisher_node,
        spawn_robot,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
