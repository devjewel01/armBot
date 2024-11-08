import rclpy
import numpy as np
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


def move_robot():
    armbot = MoveItPy(node_name="moveit_py")
    armbot_arm = armbot.get_planning_component("arm")
    armbot_gripper = armbot.get_planning_component("gripper")

    arm_state = RobotState(armbot.get_robot_model())
    gripper_state = RobotState(armbot.get_robot_model())
    arm_state.set_joint_group_positions("arm", np.array([1.57, 0.0, 0.0]))
    gripper_state.set_joint_group_positions("gripper", np.array([-0.7, 0.7]))

    armbot_arm.set_goal_state(robot_state=arm_state)
    armbot_gripper.set_goal_state(robot_state=gripper_state)

    arm_plan_result = armbot_arm.plan()
    gripper_plan_result = armbot_gripper.plan()

    if arm_plan_result and gripper_plan_result:
        get_logger("rclcp").info("Planner SUCCEED, moving the arme and the gripper")
        armbot.execute(arm_plan_result.trajectory, controllers=[])
        armbot.execute(gripper_plan_result.trajectory, controllers=[])
    else:
        get_logger("rclcp").info("One or more planners failed!")


def main():
    rclpy.init()
    move_robot()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
