#ifndef KIWI_BEHAVIOUR__BT_NODES__GRIPPER_COMMAND_ACTION_HPP_
#define KIWI_BEHAVIOUR__BT_NODES__GRIPPER_COMMAND_ACTION_HPP_

#include <string>

#include "control_msgs/action/gripper_command.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace kiwi_behaviour {

class GripperCommandAction : public nav2_behavior_tree::BtActionNode<control_msgs::action::GripperCommand> {
 public:
  GripperCommandAction(const std::string& xml_tag_name, const std::string& action_name,
                       const BT::NodeConfiguration& conf)
      : BtActionNode(xml_tag_name, action_name, conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<double>("position", "Gripper position (m)"),
        BT::InputPort<double>("max_effort", -1.0, "Max effort (-1 = no limit)"),
    });
  }

  void on_tick() override {
    getInput("position", goal_.command.position);
    getInput("max_effort", goal_.command.max_effort);
  }
};

}  // namespace kiwi_behaviour

#endif  // KIWI_BEHAVIOUR__BT_NODES__GRIPPER_COMMAND_ACTION_HPP_
