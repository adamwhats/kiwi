#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "xavbot_controller/mecanum_drive_controller.hpp"

namespace mecanum_drive_controller
{

  MecanumDriveController::MecanumDriveController() : controller_interface::ControllerInterface() {}

  controller_interface::InterfaceConfiguration MecanumDriveController::command_interface_configuration() const
  {
      controller_interface::InterfaceConfiguration command_interfaces_config;
      command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

      command_interfaces_config.names.push_back(fl_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interfaces_config.names.push_back(fr_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interfaces_config.names.push_back(rl_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interfaces_config.names.push_back(rr_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

      return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration MecanumDriveController::state_interface_configuration() const
  {
      controller_interface::InterfaceConfiguration state_interfaces_config;
      state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

      state_interfaces_config.names.push_back(fl_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interfaces_config.names.push_back(fr_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interfaces_config.names.push_back(rl_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interfaces_config.names.push_back(rr_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

      return state_interfaces_config;
  }

  controller_interface::return_type MecanumDriveController::update(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    // Get the last velocity command
    auto velocity_command = velocity_command_ptr_.readFromRT();
    if (!velocity_command || !(*velocity_command)) {
        return controller_interface::return_type::OK;
    }

    // Calculate the wheel velocities
    // Kinematics from: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
    const auto twist = *velocity_command->get();
    if (abs(twist.linear.x) > max_linear_vel_) {
      RCLCPP_WARN(get_node()->get_logger(), "Target velocity linear x component %0.3f is greater than maximum %3f", abs(twist.linear.x), max_linear_vel_);
    }
    if (abs(twist.linear.y) > max_linear_vel_) {
      RCLCPP_WARN(get_node()->get_logger(), "Target velocity linear y component %0.3f is greater than maximum %3f", abs(twist.linear.y), max_linear_vel_);
    }
    if (abs(twist.angular.z) > max_angular_vel_) {
      RCLCPP_WARN(get_node()->get_logger(), "Target velocity angular component %0.3f is greater than maximum %3f", abs(twist.angular.z), max_linear_vel_);
    }
    Eigen::Vector3d target_kinematics_ {twist.linear.x, twist.linear.y, twist.angular.z};
    Eigen::Vector4d target_vels_ = (1 / wheel_radius_) * T_inv_ * target_kinematics_;

    // Set the wheel velocities
    // TODO: Don't hardcode command_interface_ indexes 
    command_interfaces_[0].set_value(target_vels_[0]);
    command_interfaces_[1].set_value(target_vels_[1]);
    command_interfaces_[2].set_value(target_vels_[2]);
    command_interfaces_[3].set_value(target_vels_[3]);
    
    return controller_interface::return_type::OK;
  }

  CallbackReturn MecanumDriveController::on_init()
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Configure MecanumDriverController"); 

    // Create subscriptions and publishers
    velocity_command_subscription_ = get_node()->create_subscription<Twist>("/cmd_vel", 10, [this](const Twist::SharedPtr twist)
      {
        velocity_command_ptr_.writeFromNonRT(twist);
      }
    );

    // Configure controller from parameters
    fl_name_ = get_node()->get_parameter("fl_name").as_string();
    fr_name_ = get_node()->get_parameter("fr_name").as_string();
    rl_name_ = get_node()->get_parameter("rl_name").as_string();
    rr_name_ = get_node()->get_parameter("rr_name").as_string();
    if (fl_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'fl_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (fr_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'fr_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (rl_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'rl_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (rr_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'rr_name' parameter was empty");
        return CallbackReturn::ERROR;
    }

    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    wheelbase_width_ = get_node()->get_parameter("wheelbase_width").as_double();
    wheelbase_length_ = get_node()->get_parameter("wheelbase_length").as_double();
    max_wheel_rps_ = get_node()->get_parameter("max_wheel_rps").as_double();
    
    if (wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_radius' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }
    if (wheelbase_width_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheelbase_width' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }
    if (wheelbase_length_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheelbase_length' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }
    if (max_wheel_rps_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'max_wheel_rps_' parameter cannot be zero or less");
        return CallbackReturn::ERROR;
    }

    // Populate kinematics constants
    double wheel_offset_ = (wheelbase_width_ + wheelbase_length_) / 2;
    const double pi = 3.14159265358979323846;
    max_linear_vel_ = max_wheel_rps_ * wheel_radius_ * 2 * pi;
    max_angular_vel_ = (max_wheel_rps_ * wheel_radius_) / (4 * wheel_offset_);
    T_inv_ << 1, -1, -wheel_offset_,
          1, 1, wheel_offset_,
          1, 1, -wheel_offset_,
          1, -1, wheel_offset_;
    T_fw_ << 1, 1, 1, 1,
          -1, 1, 1, -1,
          -1/wheel_offset_, 1/wheel_offset_, -1/wheel_offset_, 1/wheel_offset_;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_activate(const rclcpp_lifecycle::State &)
  {
      
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_cleanup(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_error(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MecanumDriveController::on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

} // mecanum_drive_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerInterface)