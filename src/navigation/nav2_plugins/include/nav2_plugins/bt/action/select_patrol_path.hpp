#ifndef NAV2_PLUGINS__BT__ACTION__SELECT_PATROL_PATH_HPP_
#define NAV2_PLUGINS__BT__ACTION__SELECT_PATROL_PATH_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_plugins/bt/nav_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_plugins
{

class SelectPatrolPathAction : public BT::SyncActionNode
{
public:
  SelectPatrolPathAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("SelectPatrolPathAction");
  std::vector<geometry_msgs::msg::Point> goal_points_;
  std::vector<std::size_t> patrol_indices_;
  int patrol_preview_points_ = 2;
};

}  // namespace nav2_plugins

#endif  // NAV2_PLUGINS__BT__ACTION__SELECT_PATROL_PATH_HPP_
