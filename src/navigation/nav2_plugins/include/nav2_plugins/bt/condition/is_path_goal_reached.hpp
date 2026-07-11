#ifndef NAV2_PLUGINS__BT__CONDITION__IS_PATH_GOAL_REACHED_HPP_
#define NAV2_PLUGINS__BT__CONDITION__IS_PATH_GOAL_REACHED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_plugins/bt/nav_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_plugins
{

class IsPathGoalReachedCondition : public BT::SimpleConditionNode
{
public:
  IsPathGoalReachedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsPathGoalReachedCondition");
  double path_tolerance_ = 0.2;
};

}  // namespace nav2_plugins

#endif  // NAV2_PLUGINS__BT__CONDITION__IS_PATH_GOAL_REACHED_HPP_
