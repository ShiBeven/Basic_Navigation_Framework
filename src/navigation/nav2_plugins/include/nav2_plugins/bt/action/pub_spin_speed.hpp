#ifndef NAV2_PLUGINS__BT__ACTION__PUB_SPIN_SPEED_HPP_
#define NAV2_PLUGINS__BT__ACTION__PUB_SPIN_SPEED_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_plugins
{

class PublishSpinSpeedAction
: public BT::RosTopicPubStatefulActionNode<example_interfaces::msg::Float32>
{
public:
  PublishSpinSpeedAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(example_interfaces::msg::Float32 & msg) override;

  bool setHaltMessage(example_interfaces::msg::Float32 & msg) override;
};

}  // namespace nav2_plugins

#endif  // NAV2_PLUGINS__BT__ACTION__PUB_SPIN_SPEED_HPP_
