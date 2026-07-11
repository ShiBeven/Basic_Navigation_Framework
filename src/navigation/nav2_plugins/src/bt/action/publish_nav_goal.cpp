#include "nav2_plugins/bt/action/publish_nav_goal.hpp"

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_plugins
{

PublishNavGoalAction::PublishNavGoalAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config), node_(getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, 10);
  node_->get_parameter(
    "nav.decision_config.goal_position_tolerance", goal_position_tolerance_);
}

BT::NodeStatus PublishNavGoalAction::tick()
{
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(logger_, "PublishNavGoal did not receive a valid goal input");
    return BT::NodeStatus::FAILURE;
  }

  std::string topic_name = topic_name_;
  getInput("topic_name", topic_name);
  if (!goal_pub_ || topic_name != topic_name_) {
    topic_name_ = topic_name;
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, 10);
    has_last_goal_ = false;
    burst_republish_remaining_ = 0;
  }

  geometry_msgs::msg::PoseStamped goal_msg = *goal;
  goal_msg.header.stamp = node_->now();
  if (goal_msg.header.frame_id.empty()) {
    std::string goal_frame = "map";
    getInput("goal_frame", goal_frame);
    goal_msg.header.frame_id = goal_frame;
  }

  if (isSameGoal(goal_msg)) {
    if (burst_republish_remaining_ <= 0) {
      return BT::NodeStatus::SUCCESS;
    }

    goal_pub_->publish(goal_msg);
    --burst_republish_remaining_;
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    logger_, "Publish nav goal to %s: (%.2f, %.2f, %.2f)", topic_name_.c_str(),
    goal_msg.pose.position.x, goal_msg.pose.position.y,
    tf2::getYaw(goal_msg.pose.orientation));
  goal_pub_->publish(goal_msg);
  last_goal_ = goal_msg;
  has_last_goal_ = true;
  burst_republish_remaining_ = 1;
  return BT::NodeStatus::SUCCESS;
}

bool PublishNavGoalAction::isSameGoal(const geometry_msgs::msg::PoseStamped & goal) const
{
  if (!has_last_goal_) {
    return false;
  }

  const auto distance = std::sqrt(
    squaredDistance(last_goal_.pose.position, goal.pose.position));
  const auto last_yaw = tf2::getYaw(last_goal_.pose.orientation);
  const auto goal_yaw = tf2::getYaw(goal.pose.orientation);

  return distance <= goal_position_tolerance_ &&
         std::fabs(normalizeAngle(last_yaw - goal_yaw)) <= 0.05;
}

BT::PortsList PublishNavGoalAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "{decision_goal_pose}", "Goal pose published to Nav2"),
    BT::InputPort<std::string>("topic_name", "goal_pose", "Goal pose topic name"),
    BT::InputPort<std::string>(
      "goal_frame", "map", "Frame id used when the goal pose has no frame set")};
}

}  // namespace nav2_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_plugins::PublishNavGoalAction>(
    "PublishNavGoal");
}
