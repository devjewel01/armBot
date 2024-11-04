import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
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

    # Robot description as a parameter
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )

    # Node for publishing robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Node for joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Node for launching RViz with a pre-configured display file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(armbot_description_dir, 'rviz', 'display.rviz')],
    )

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
