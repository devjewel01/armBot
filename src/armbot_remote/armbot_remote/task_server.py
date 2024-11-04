#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from armbot_msgs.action import ArmbotTask  # Updated for armbot
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Starting the Armbot Task Server")
        self.action_server = ActionServer(
            self, ArmbotTask, "armbot_task", self.goalCallback  # Updated action and name
        )

        # MoveIt 2 Interface
        self.armbot = MoveItPy(node_name="moveit_py")
        self.armbot_arm = self.armbot.get_planning_component("arm")
        self.armbot_gripper = self.armbot.get_planning_component("gripper")

    def goalCallback(self, goal_handle):
        self.get_logger().info(
            "Received goal request with task number %d" % goal_handle.request.task_number
        )

        arm_state = RobotState(self.armbot.get_robot_model())
        gripper_state = RobotState(self.armbot.get_robot_model())

        arm_joint_goal = []
        gripper_joint_goal = []

        if goal_handle.request.task_number == 0:
            arm_joint_goal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Adjusted for 6-DOF arm
            gripper_joint_goal = np.array([-0.7, 0.7])
        elif goal_handle.request.task_number == 1:
            arm_joint_goal = np.array([-1.14, -0.6, -0.07, 0.0, 0.0, 0.0])  # Example values
            gripper_joint_goal = np.array([0.0, 0.0])
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = np.array([-1.57, 0.0, -0.9, 0.0, 0.5, 0.3])  # Example values
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().error("Invalid Task Number")
            return

        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)

        self.armbot_arm.set_start_state_to_current_state()
        self.armbot_gripper.set_start_state_to_current_state()

        self.armbot_arm.set_goal_state(robot_state=arm_state)
        self.armbot_gripper.set_goal_state(robot_state=gripper_state)

        arm_plan_result = self.armbot_arm.plan()
        gripper_plan_result = self.armbot_gripper.plan()

        if arm_plan_result and gripper_plan_result:
            self.get_logger().info("Planner SUCCEEDED, moving the arm and the gripper")
            self.armbot.execute(arm_plan_result.trajectory, controllers=[])
            self.armbot.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().info("One or more planners failed!")

        self.get_logger().info("Goal succeeded")
        goal_handle.succeed()
        result = ArmbotTask.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)

if __name__ == "__main__":
    main()
