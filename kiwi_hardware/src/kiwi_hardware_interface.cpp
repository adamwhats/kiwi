#include "kiwi_hardware_interface.hpp"
#include "motor2040_comms.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kiwi_hardware_interface
{
CallbackReturn KiwiHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("KiwiHardware"), "Initializing...");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) 
  {
    return CallbackReturn::ERROR;
  }

  cfg_.fl_name = info_.hardware_parameters["fl_name"];
  cfg_.fr_name = info_.hardware_parameters["fr_name"];
  cfg_.rl_name = info_.hardware_parameters["rl_name"];
  cfg_.rr_name = info_.hardware_parameters["rr_name"];
  cfg_.fl_port = std::stoi(info_.hardware_parameters["fl_port"]);
  cfg_.fr_port = std::stoi(info_.hardware_parameters["fr_port"]);
  cfg_.rl_port = std::stoi(info_.hardware_parameters["rl_port"]);
  cfg_.rr_port = std::stoi(info_.hardware_parameters["rr_port"]);
  cfg_.fl_direction = std::stoi(info_.hardware_parameters["fl_direction"]);
  cfg_.fr_direction = std::stoi(info_.hardware_parameters["fr_direction"]);
  cfg_.rl_direction = std::stoi(info_.hardware_parameters["rl_direction"]);
  cfg_.rr_direction = std::stoi(info_.hardware_parameters["rr_direction"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Expects exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KiwiHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KiwiHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KiwiHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KiwiHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KiwiHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("KiwiHardware"), "Initialized!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KiwiHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      *cfg_.wheel_names[i], hardware_interface::HW_IF_POSITION, &wheels_.positions[*cfg_.wheel_ports[i]]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      *cfg_.wheel_names[i], hardware_interface::HW_IF_VELOCITY, &wheels_.velocities[*cfg_.wheel_ports[i]]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KiwiHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      *cfg_.wheel_names[i], hardware_interface::HW_IF_VELOCITY, &wheels_.cmd[*cfg_.wheel_ports[i]]));
  }
  return command_interfaces;
}

CallbackReturn KiwiHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("KiwiHardware"), "Activating...");
  motor_controller_.connect(cfg_.device, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("KiwiHardware"), "Activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KiwiHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("KiwiHardware"), "Deactivating...");
  motor_controller_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("KiwiHardware"), "Deactivated!");
  return CallbackReturn::SUCCESS;
}

return_type KiwiHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // Get raw values from motor controller
  const auto raw_positions = motor_controller_.get_positions();
  const auto raw_velocities = motor_controller_.get_velocities();
  
  // Apply direction correction and store in wheels_ structure
  const auto corrected_positions = apply_direction_correction(raw_positions);
  const auto corrected_velocities = apply_direction_correction(raw_velocities);
  std::copy(corrected_positions.begin(), corrected_positions.end(), wheels_.positions.begin());
  std::copy(corrected_velocities.begin(), corrected_velocities.end(), wheels_.velocities.begin());

  return hardware_interface::return_type::OK;
}

return_type KiwiHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  const auto direction_corrected_vels = apply_direction_correction(wheels_.cmd);
  motor_controller_.set_velocities(direction_corrected_vels);
  return hardware_interface::return_type::OK;
}

std::array<double, 4> KiwiHardware::apply_direction_correction(const std::array<double, 4>& values) const
{
  std::array<double, 4> corrected_values;
  for (size_t i = 0; i < 4; ++i)
  {
    corrected_values[i] = values[i] * *cfg_.wheel_directions[*cfg_.wheel_ports[i]];
  }
  return corrected_values;
}

}  // namespace kiwi_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kiwi_hardware_interface::KiwiHardware, hardware_interface::SystemInterface)