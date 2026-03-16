#ifndef KIWI_BEHAVIOUR__BT_NODES__SELECT_GRASP_TARGET_ACTION_HPP_
#define KIWI_BEHAVIOUR__BT_NODES__SELECT_GRASP_TARGET_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kiwi_interfaces/action/select_grasp_target.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace kiwi_behaviour {

class SelectGraspTargetAction
    : public nav2_behavior_tree::BtActionNode<kiwi_interfaces::action::SelectGraspTarget> {
 public:
  SelectGraspTargetAction(const std::string& xml_tag_name, const std::string& action_name,
                          const BT::NodeConfiguration& conf)
      : BtActionNode(xml_tag_name, action_name, conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<double>("standoff_distance", 0.08, "Standoff metres"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("base_pose"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("grasp_pose"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pre_pick_pose"),
    });
  }

  void on_tick() override {
    double s;
    getInput("standoff_distance", s);
    goal_.standoff_distance = static_cast<float>(s);
  }

  BT::NodeStatus on_success() override {
    setOutput("base_pose", this->result_.result->base_pose);
    setOutput("grasp_pose", this->result_.result->grasp_pose);
    setOutput("pre_pick_pose", this->result_.result->pre_pick_pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace kiwi_behaviour

#endif  // KIWI_BEHAVIOUR__BT_NODES__SELECT_GRASP_TARGET_ACTION_HPP_
