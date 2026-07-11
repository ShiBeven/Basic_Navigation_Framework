#include "nav2_plugins/bt/action/select_path_goal_pose.hpp"

namespace nav2_plugins
{

BT::NodeStatus SelectPathGoalPoseAction::tick()
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  if (!path || path->poses.empty()) {
    RCLCPP_ERROR(logger_, "SelectPathGoalPose did not receive a valid path input");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("goal", path->poses.back());
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SelectPathGoalPoseAction::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Current decision path"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "{decision_goal_pose}", "Goal pose extracted from the current path")};
}

}  // namespace nav2_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_plugins::SelectPathGoalPoseAction>("SelectPathGoalPose");
}
