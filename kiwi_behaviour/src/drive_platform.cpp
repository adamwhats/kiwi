#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "kiwi_interfaces/action/drive_platform.hpp"

using namespace std::placeholders;

namespace kiwi_behaviour
{
class DrivePlatformServer : public rclcpp::Node
{
public:
  using Drive = kiwi_interfaces::action::DrivePlatform;
  using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

  DrivePlatformServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("drive_platform_server", options)
  {
    this->action_server_ = rclcpp_action::create_server<Drive>(
      this,
      "/drive_platform",
      std::bind(&DrivePlatformServer::handle_goal, this, _1, _2),
      std::bind(&DrivePlatformServer::handle_cancel, this, _1),
      std::bind(&DrivePlatformServer::handle_accepted, this, _1));
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp_action::Server<Drive>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Drive::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received DrivePlatform request");
    (void)uuid;
    if (!goal->topic.empty())
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_WARN(this->get_logger(), "Request 'topic' field is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDrive> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    geometry_msgs::msg::Twist empty_twist;
    twist_pub->publish(empty_twist);
    return rclcpp_action::CancelResponse::ACCEPT;

  }

  void handle_accepted(const std::shared_ptr<GoalHandleDrive> goal_handle)
  {
    std::thread{std::bind(&DrivePlatformServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDrive> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Drive::Result>();
     
    this->create_publisher<geometry_msgs::msg::Twist>(goal->topic, rclcpp::SystemDefaultsQoS());

    rclcpp::Time finish_time;
    finish_time = this->get_clock()->now() + goal->duration;
    while (this->get_clock()->now() < finish_time)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Otherwise continue publishing
      twist_pub->publish(goal->twist);
    }

    goal_handle->succeed(result);
  }
};  // class DrivePlatformServer
} // namespace kiwi_behaviour

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiwi_behaviour::DrivePlatformServer)