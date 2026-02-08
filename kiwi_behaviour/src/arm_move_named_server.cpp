#include <moveit/move_group_interface/move_group_interface.h>

#include "kiwi_interfaces/action/arm_move_named.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;

namespace kiwi_behaviour {

class ArmMoveNamedServer : public rclcpp::Node {
 public:
  using ArmMoveNamed = kiwi_interfaces::action::ArmMoveNamed;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ArmMoveNamed>;

  explicit ArmMoveNamedServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("arm_move_named_server", options) {
    action_server_ = rclcpp_action::create_server<ArmMoveNamed>(
        this, "~/cmd", std::bind(&ArmMoveNamedServer::handle_goal, this, _1, _2),
        std::bind(&ArmMoveNamedServer::handle_cancel, this, _1),
        std::bind(&ArmMoveNamedServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "ArmMoveNamed action server ready");
  }

 private:
  rclcpp_action::Server<ArmMoveNamed>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const ArmMoveNamed::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Goal received: group='%s' target='%s'", goal->group_name.c_str(),
                goal->target_name.c_str());
    if (goal->group_name.empty() || goal->target_name.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting: group_name and target_name required");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&ArmMoveNamedServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ArmMoveNamed::Result>();
    auto feedback = std::make_shared<ArmMoveNamed::Feedback>();

    // Connect to move_group (blocks until services are available)
    feedback->status = "Connecting to move_group...";
    goal_handle->publish_feedback(feedback);

    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), goal->group_name);

    if (!move_group.setNamedTarget(goal->target_name)) {
      result->success = false;
      result->message = "Unknown target '" + goal->target_name + "' for group '" + goal->group_name + "'";
      goal_handle->abort(result);
      return;
    }

    // Plan
    feedback->status = "Planning...";
    goal_handle->publish_feedback(feedback);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled before execution";
      goal_handle->canceled(result);
      return;
    }

    // Execute
    feedback->status = "Executing...";
    goal_handle->publish_feedback(feedback);

    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = false;
      result->message = "Execution failed";
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    result->message = "Reached '" + goal->target_name + "'";
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
  }
};

}  // namespace kiwi_behaviour

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiwi_behaviour::ArmMoveNamedServer)
