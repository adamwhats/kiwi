#ifndef KIWI_BEHAVIOUR__BT_NODES__NAVIGATE_TO_POSE_ACTION_HPP_
#define KIWI_BEHAVIOUR__BT_NODES__NAVIGATE_TO_POSE_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace kiwi_behaviour {

class NavigateToPoseAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose> {
 public:
  NavigateToPoseAction(const std::string& xml_tag_name, const std::string& action_name,
                       const BT::NodeConfiguration& conf)
      : BtActionNode(xml_tag_name, action_name, conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target pose"),
    });
  }

  void on_tick() override {
    getInput("goal", goal_.pose);
    goal_.behavior_tree = "";  // Use Nav2's default BT
  }
};

}  // namespace kiwi_behaviour

#endif  // KIWI_BEHAVIOUR__BT_NODES__NAVIGATE_TO_POSE_ACTION_HPP_
