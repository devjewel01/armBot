import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    armbot_description_dir = get_package_share_directory('armbot_description')

    # Argument for model file path
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(armbot_description_dir, 'urdf', 'armbot.urdf.xacro'),
        description='Absolute path to robot URDF file'
    )

    # Robot description parameter
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz Node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(armbot_description_dir, 'rviz', 'display.rviz')],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
