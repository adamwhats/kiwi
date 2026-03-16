#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "kiwi_interfaces/action/execute_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "bt_nodes/arm_move_named_action.hpp"
#include "bt_nodes/gripper_command_action.hpp"
#include "bt_nodes/navigate_to_pose_action.hpp"
#include "bt_nodes/select_grasp_target_action.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace kiwi_behaviour
{

class TaskExecutor : public rclcpp::Node
{
public:
  using ExecuteTask = kiwi_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit TaskExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("task_executor", options)
  {
    declare_parameter("tree_xml_file", "");
    declare_parameter("server_timeout_ms", 20000);
    declare_parameter("bt_loop_duration_ms", 10);
    declare_parameter("wait_for_service_timeout_ms", 1000);

    tree_xml_file_ = get_parameter("tree_xml_file").as_string();
    if (tree_xml_file_.empty()) {
      throw std::runtime_error("tree_xml_file parameter is required");
    }

    register_bt_nodes();

    action_server_ = rclcpp_action::create_server<ExecuteTask>(
      this, "~/execute_task",
      std::bind(&TaskExecutor::handle_goal, this, _1, _2),
      std::bind(&TaskExecutor::handle_cancel, this, _1),
      std::bind(&TaskExecutor::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Ready (tree: %s)", tree_xml_file_.c_str());
  }

private:
  void register_bt_nodes()
  {
    BT::NodeBuilder arm_builder =
      [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<ArmMoveNamedAction>(name, "/bt/arm_move_named", config);
      };
    factory_.registerBuilder<ArmMoveNamedAction>("ArmMoveNamed", arm_builder);

    BT::NodeBuilder gripper_builder =
      [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<GripperCommandAction>(name, "/gripper_controller/gripper_cmd", config);
      };
    factory_.registerBuilder<GripperCommandAction>("GripperCommand", gripper_builder);

    BT::NodeBuilder nav_builder =
      [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<NavigateToPoseAction>(name, "/navigate_to_pose", config);
      };
    factory_.registerBuilder<NavigateToPoseAction>("NavigateToPose", nav_builder);

    BT::NodeBuilder grasp_builder =
      [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<SelectGraspTargetAction>(
          name, "/grasp_generator/select_target", config);
      };
    factory_.registerBuilder<SelectGraspTargetAction>("SelectGraspTarget", grasp_builder);
  }

  BT::Blackboard::Ptr create_blackboard()
  {
    auto bb = BT::Blackboard::create();
    bb->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
    bb->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(get_parameter("server_timeout_ms").as_int()));
    bb->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(get_parameter("bt_loop_duration_ms").as_int()));
    bb->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      std::chrono::milliseconds(get_parameter("wait_for_service_timeout_ms").as_int()));
    return bb;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteTask::Goal>)
  {
    if (executing_.load()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: tree already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&TaskExecutor::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    executing_.store(true);
    cancel_requested_.store(false);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ExecuteTask::Result>();

    try {
      auto blackboard = create_blackboard();
      blackboard->set("target_pose", goal->target_pose);

      auto tree = factory_.createTreeFromFile(tree_xml_file_, blackboard);

      auto loop_duration = std::chrono::milliseconds(
        get_parameter("bt_loop_duration_ms").as_int());

      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        if (cancel_requested_.load()) {
          tree.rootNode()->halt();
          result->success = false;
          result->message = "Canceled";
          goal_handle->canceled(result);
          executing_.store(false);
          return;
        }

        status = tree.tickRoot();
        if (status == BT::NodeStatus::RUNNING) {
          std::this_thread::sleep_for(loop_duration);
        }
      }

      result->success = (status == BT::NodeStatus::SUCCESS);
      result->message = result->success ? "Task completed" : "Task failed";
      if (result->success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (const std::exception & e) {
      result->success = false;
      result->message = std::string("Exception: ") + e.what();
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    }

    executing_.store(false);
  }

  BT::BehaviorTreeFactory factory_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  std::string tree_xml_file_;
  std::atomic<bool> executing_{false};
  std::atomic<bool> cancel_requested_{false};
};

}  // namespace kiwi_behaviour

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiwi_behaviour::TaskExecutor)
