#include "nav2_plugins/bt/condition/is_path_goal_reached.hpp"

namespace nav2_plugins
{

IsPathGoalReachedCondition::IsPathGoalReachedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsPathGoalReachedCondition::tickCondition, this), config)
{
  const auto node = getNodeFromBlackboard(*this);
  node->get_parameter("nav.decision_config.path_tolerance", path_tolerance_);
}

BT::NodeStatus IsPathGoalReachedCondition::tickCondition()
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
  if (!path || !current_pose || path->poses.empty()) {
    RCLCPP_DEBUG(logger_, "Path or current pose is not available");
    return BT::NodeStatus::FAILURE;
  }

  return isPathGoalReached(*current_pose, *path, path_tolerance_) ?
           BT::NodeStatus::SUCCESS :
           BT::NodeStatus::FAILURE;
}

BT::PortsList IsPathGoalReachedCondition::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Current decision path"),
    BT::InputPort<bool>("goal_succeeded", "Whether NavigateThroughPoses already reported success for this path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "current_pose", "{decision_current_pose}", "Current navigation feedback pose")};
}

}  // namespace nav2_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_plugins::IsPathGoalReachedCondition>(
    "IsPathGoalReached");
}
