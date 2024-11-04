#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SliderControl(Node):

    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub_ = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

        # Define joint names for the arm and gripper
        self.arm_joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        self.gripper_joint_names = ["joint_6"]

    def sliderCallback(self, msg):
        if len(msg.position) < 6:
            self.get_logger().warn("Received joint command message with insufficient positions.")
            return

        # Prepare JointTrajectory messages for arm and gripper
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()
        arm_controller.joint_names = self.arm_joint_names
        gripper_controller.joint_names = self.gripper_joint_names

        # Set positions for arm joints
        arm_goal = JointTrajectoryPoint()
        arm_goal.positions = msg.position[:5]  # Assuming first 5 positions are for arm joints
        arm_goal.time_from_start.sec = 1  # Example timing

        # Set position for gripper joint (assuming it's the 6th joint in msg.position)
        gripper_goal = JointTrajectoryPoint()
        gripper_goal.positions = [msg.position[5]]  # Assuming 6th position is for gripper joint
        gripper_goal.time_from_start.sec = 1  # Example timing

        # Append the trajectory points to respective controllers
        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)

        # Publish the JointTrajectory messages
        self.arm_pub_.publish(arm_controller)
        self.gripper_pub_.publish(gripper_controller)
        self.get_logger().info("Published joint trajectory commands.")

def main():
    rclpy.init()

    simple_publisher = SliderControl()
    rclpy.spin(simple_publisher)
    
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
