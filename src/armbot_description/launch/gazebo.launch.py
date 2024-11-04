import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directory for armbot_description
    armbot_description_dir = get_package_share_directory('armbot_description')

    # Declare arguments for model file path and use_sim_time
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(armbot_description_dir, 'urdf', 'armbot.urdf.xacro'),
        description='Absolute path to robot URDF/Xacro file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # Set environment variable for Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(armbot_description_dir).parent.resolve())]
    )

    # Generate robot description as a parameter
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )

    # Node for robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[('gz_args', ['-v 4 -r empty.sdf --physics-engine gz-physics-bullet-featherstone-plugin'])]
    )

    # Node for spawning the robot entity in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'armbot']
    )

    # Node for bridging topics between ROS 2 and Gazebo
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
