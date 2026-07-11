#ifndef NAV2_PLUGINS__BT__ACTION__PUBLISH_NAV_GOAL_HPP_
#define NAV2_PLUGINS__BT__ACTION__PUBLISH_NAV_GOAL_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_plugins/bt/nav_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_plugins
{

class PublishNavGoalAction : public BT::SyncActionNode
{
public:
  PublishNavGoalAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  bool isSameGoal(const geometry_msgs::msg::PoseStamped & goal) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("PublishNavGoalAction");
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  geometry_msgs::msg::PoseStamped last_goal_;
  std::string topic_name_ = "goal_pose";
  double goal_position_tolerance_ = 0.1;
  bool has_last_goal_ = false;
  int burst_republish_remaining_ = 0;
};

}  // namespace nav2_plugins

#endif  // NAV2_PLUGINS__BT__ACTION__PUBLISH_NAV_GOAL_HPP_
