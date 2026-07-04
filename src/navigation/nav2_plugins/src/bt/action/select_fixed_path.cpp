#include "nav2_plugins/bt/action/select_fixed_path.hpp"

namespace nav2_plugins
{

SelectFixedPathAction::SelectFixedPathAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  const auto node = getNodeFromBlackboard(*this);

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;

  node->get_parameter("nav.goal_points.x", xs);
  node->get_parameter("nav.goal_points.y", ys);
  node->get_parameter("nav.goal_points.z", zs);
  goal_points_ = buildGoalPoints(xs, ys, zs);
}

BT::NodeStatus SelectFixedPathAction::tick()
{
  int target_index = -1;
  if (!getInput("target_index", target_index) || target_index < 0) {
    RCLCPP_ERROR(logger_, "SelectFixedPath did not receive a valid target_index");
    return BT::NodeStatus::FAILURE;
  }

  const auto index = validateIndex(
    static_cast<std::size_t>(target_index), goal_points_.size(), "target_index");
  nav_msgs::msg::Path path =
    buildPathFromIndices(goal_points_, {index});

  setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SelectFixedPathAction::providedPorts()
{
  return {
    BT::InputPort<int>("target_index", -1, "Fixed target index"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Fixed decision path")};
}

}  // namespace nav2_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_plugins::SelectFixedPathAction>("SelectFixedPath");
}
