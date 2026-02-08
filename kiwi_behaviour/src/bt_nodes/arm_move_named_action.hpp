#ifndef KIWI_BEHAVIOUR__BT_NODES__ARM_MOVE_NAMED_ACTION_HPP_
#define KIWI_BEHAVIOUR__BT_NODES__ARM_MOVE_NAMED_ACTION_HPP_

#include <string>

#include "kiwi_interfaces/action/arm_move_named.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace kiwi_behaviour {

class ArmMoveNamedAction : public nav2_behavior_tree::BtActionNode<kiwi_interfaces::action::ArmMoveNamed> {
 public:
  ArmMoveNamedAction(const std::string& xml_tag_name, const std::string& action_name, const BT::NodeConfiguration& conf)
      : BtActionNode(xml_tag_name, action_name, conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("group_name", "combined", "MoveIt planning group"),
        BT::InputPort<std::string>("target_name", "Named target in SRDF"),
    });
  }

  void on_tick() override {
    getInput("group_name", goal_.group_name);
    getInput("target_name", goal_.target_name);
  }
};

}  // namespace kiwi_behaviour

#endif  // KIWI_BEHAVIOUR__BT_NODES__ARM_MOVE_NAMED_ACTION_HPP_
