#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from armbot_msgs.action import ArmbotTask
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Starting the Server")
        self.action_server = ActionServer(
            self, ArmbotTask, "task_server", self.goalCallback
        )

    def goalCallback(self, goal_handle):
        self.get_logger().info(
            "Received goal request with id %d" % goal_handle.request.task_number
        )

        # MoveIt 2 Interface
        armbot = MoveItPy(node_name="moveit_py")
        armbot_arm = armbot.get_planning_component("arm")
        armbot_gripper = armbot.get_planning_component("gripper")

        arm_state = RobotState(armbot.get_robot_model())
        gripper_state = RobotState(armbot.get_robot_model())

        arm_joint_goal = []
        gripper_joint_goal = []

        if goal_handle.request.task_number == 0:
            arm_joint_goal = np.array([0.0, 0.0, 0.0])
            gripper_joint_goal = np.array([-0.7, 0.7])
        elif goal_handle.request.task_number == 1:
            arm_joint_goal = np.array([-1.14, -0.6, -0.07])
            gripper_joint_goal = np.array([0.0, 0.0])
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = np.array([-1.57,0.0,-0.9])
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().error("Invalid Task Number")
            return

        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)

        armbot_arm.set_goal_state(robot_state=arm_state)
        armbot_gripper.set_goal_state(robot_state=gripper_state)

        arm_plan_result = armbot_arm.plan()
        gripper_plan_result = armbot_gripper.plan()

        if arm_plan_result and gripper_plan_result:
            self.get_logger().info("Planner SUCCEED, moving the arme and the gripper")
            armbot.execute(arm_plan_result.trajectory, controllers=[])
            armbot.execute(gripper_plan_result.trajectory, controllers=[])
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
