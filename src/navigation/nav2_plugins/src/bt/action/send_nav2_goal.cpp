#include "nav2_plugins/bt/action/send_nav2_goal.hpp"

#include "nav2_plugins/bt/custom_types.hpp"

namespace nav2_plugins
{

SendNav2GoalAction::SendNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool SendNav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  auto receive_goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!receive_goal) {
    RCLCPP_ERROR(logger(), "SendNav2GoalAction did not receive a valid goal input");
    return false;
  }

  // Prefer the frame carried by the incoming goal; otherwise fall back to the
  // configurable `goal_frame` port (default "map") so the node is reusable in
  // projects whose global frame is not named "map".
  std::string goal_frame = "map";
  getInput("goal_frame", goal_frame);
  goal.pose.header.frame_id =
    receive_goal->header.frame_id.empty() ? goal_frame : receive_goal->header.frame_id;
  goal.pose.header.stamp = now();
  goal.pose.pose = receive_goal->pose;

  return true;
}

BT::NodeStatus SendNav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Navigation succeeded!");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "Navigation aborted by server");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "Navigation canceled");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown navigation result code: %d", static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendNav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Distance remaining: %f", feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

void SendNav2GoalAction::onHalt() { RCLCPP_INFO(logger(), "SendNav2GoalAction has been halted."); }

BT::NodeStatus SendNav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "SendNav2GoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::PortsList SendNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<std::string>(
      "goal_frame", "map", "Frame id used when the goal pose has no frame set"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace nav2_plugins

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(nav2_plugins::SendNav2GoalAction, "SendNav2Goal");
