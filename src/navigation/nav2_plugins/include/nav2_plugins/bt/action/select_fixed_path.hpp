#ifndef NAV2_PLUGINS__BT__ACTION__SELECT_FIXED_PATH_HPP_
#define NAV2_PLUGINS__BT__ACTION__SELECT_FIXED_PATH_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_plugins/bt/nav_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_plugins
{

class SelectFixedPathAction : public BT::SyncActionNode
{
public:
  SelectFixedPathAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("SelectFixedPathAction");
  std::vector<geometry_msgs::msg::Point> goal_points_;
};

}  // namespace nav2_plugins

#endif  // NAV2_PLUGINS__BT__ACTION__SELECT_FIXED_PATH_HPP_
