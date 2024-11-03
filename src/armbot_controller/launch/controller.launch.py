import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration for simulation mode
    is_sim = LaunchConfiguration("is_sim")

    # Declare the argument for simulation mode
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
        description="Flag to indicate if the robot is running in simulation mode"
    )

    # Generate the robot description from the Xacro file
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("armbot_description"),
                    "urdf",
                    "armbot.urdf.xacro",
                ),
                " is_sim:=", is_sim
            ]
        ),
        value_type=str,
    )

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        condition=UnlessCondition(is_sim),
    )

    # Controller manager node for real robot use
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("armbot_controller"),
                "config",
                "armbot_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

    # Spawners for controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Return the complete launch description
    return LaunchDescription(
        [
            is_sim_arg,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
