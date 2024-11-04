#include "armbot_controller/armbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>

namespace armbot_controller
{

// Helper function to format a command string with zero-padding
std::string ArmbotInterface::format_command(const std::string& prefix, int value) const
{
  std::ostringstream oss;
  oss << prefix << std::setw(3) << std::setfill('0') << value << ",";
  return oss.str();
}

ArmbotInterface::ArmbotInterface() = default;
ArmbotInterface::~ArmbotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArmbotInterface"),
                          "Exception while closing port " << port_ << ": " << e.what());
    }
  }
}

CallbackReturn ArmbotInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  auto result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range&)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ArmbotInterface"), "No Serial Port provided! Aborting.");
    return CallbackReturn::FAILURE;
  }

  position_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  prev_position_commands_.resize(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn ArmbotInterface::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmbotInterface"), "Activating hardware...");

  position_commands_.assign(info_.joints.size(), 0.0);
  prev_position_commands_.assign(info_.joints.size(), 0.0);
  position_states_.assign(info_.joints.size(), 0.0);

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArmbotInterface"),
                        "Failed to open port " << port_ << ": " << e.what());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArmbotInterface"), "Hardware activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmbotInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmbotInterface"), "Deactivating hardware...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArmbotInterface"),
                          "Exception while closing port " << port_ << ": " << e.what());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ArmbotInterface"), "Hardware deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmbotInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  // Simulate perfect execution by mirroring commands to states
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmbotInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  if (position_commands_ == prev_position_commands_)
  {
    return hardware_interface::return_type::OK;  // No change in commands, skip sending
  }

  std::string msg;
  msg += format_command("b", static_cast<int>(((position_commands_[0] + M_PI / 2) * 180) / M_PI));
  msg += format_command("s", 180 - static_cast<int>(((position_commands_[1] + M_PI / 2) * 180) / M_PI));
  msg += format_command("e", static_cast<int>(((position_commands_[2] + M_PI / 2) * 180) / M_PI));
  msg += format_command("w", static_cast<int>(((position_commands_[3] + M_PI / 2) * 180) / M_PI));
  msg += format_command("r", static_cast<int>(((position_commands_[4] + M_PI / 2) * 180) / M_PI));
  msg += format_command("g", static_cast<int>((-position_commands_[5] * 180) / (M_PI / 2)));

  try
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ArmbotInterface"), "Sending command: " << msg);
    arduino_.Write(msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArmbotInterface"),
                        "Failed to send message to port " << port_ << ": " << e.what());
    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;
  return hardware_interface::return_type::OK;
}

}  // namespace armbot_controller

PLUGINLIB_EXPORT_CLASS(armbot_controller::ArmbotInterface, hardware_interface::SystemInterface)
