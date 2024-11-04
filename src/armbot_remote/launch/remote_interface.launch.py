import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
        description="Flag to indicate if running in simulation mode"
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
        description="Flag to indicate if Python version of the task server should be used"
    )

    use_python = LaunchConfiguration("use_python")
    is_sim = LaunchConfiguration("is_sim")

    task_server_node = Node(
        package="armbot_remote",
        executable="task_server_node",
        condition=UnlessCondition(use_python),
        parameters=[{"use_sim_time": is_sim}]
    )

    moveit_config = (
        MoveItConfigsBuilder("armbot", package_name="armbot_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("armbot_description"),
            "urdf",
            "armbot.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/armbot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()
    )

    task_server_node_py = Node(
        package="armbot_remote",
        executable="task_server.py",
        condition=IfCondition(use_python),
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": is_sim}]
    )

    alexa_interface_node = Node(
        package="armbot_remote",
        executable="alexa_interface.py",
        parameters=[{"use_sim_time": is_sim}]
    )

    return LaunchDescription([
        use_python_arg,
        is_sim_arg,
        task_server_node,
        task_server_node_py,
        alexa_interface_node
    ])
