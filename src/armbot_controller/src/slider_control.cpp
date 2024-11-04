#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SliderControl : public rclcpp::Node
{
public:
  SliderControl() : Node("slider_control"),
                    arm_joint_names_{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"},
                    gripper_joint_names_{"joint_6"}
  {
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands", 10, std::bind(&SliderControl::sliderCallback, this, _1));
    arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("arm_controller/joint_trajectory", 10);
    gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("gripper_controller/joint_trajectory", 10);
    RCLCPP_INFO(get_logger(), "Slider Control Node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> gripper_joint_names_;

  void sliderCallback(const sensor_msgs::msg::JointState &msg) const
  {
    if (msg.position.size() < arm_joint_names_.size() + gripper_joint_names_.size())
    {
      RCLCPP_WARN(get_logger(), "Received joint state message with insufficient positions. Ignoring...");
      return;
    }

    trajectory_msgs::msg::JointTrajectory arm_command, gripper_command;
    arm_command.joint_names = arm_joint_names_;
    gripper_command.joint_names = gripper_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint arm_goal, gripper_goal;
    
    // Copy the positions for arm joints
    arm_goal.positions.insert(arm_goal.positions.end(), msg.position.begin(), msg.position.begin() + arm_joint_names_.size());
    // Copy the position for the gripper joint
    gripper_goal.positions.push_back(msg.position.at(arm_joint_names_.size()));  // Assumes gripper is the next joint

    arm_command.points.push_back(arm_goal);
    gripper_command.points.push_back(gripper_goal);

    arm_pub_->publish(arm_command);
    gripper_pub_->publish(gripper_command);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SliderControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
