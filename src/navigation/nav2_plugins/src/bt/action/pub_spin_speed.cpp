#include "nav2_plugins/bt/action/pub_spin_speed.hpp"

namespace nav2_plugins
{

PublishSpinSpeedAction::PublishSpinSpeedAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishSpinSpeedAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("spin_speed", 0.0, "Angular Z velocity (rad/s)"),
  });
}

bool PublishSpinSpeedAction::setMessage(example_interfaces::msg::Float32 & msg)
{
  double spin_speed = 0.0;
  getInput("spin_speed", spin_speed);

  msg.data = spin_speed;

  return true;
}

bool PublishSpinSpeedAction::setHaltMessage(example_interfaces::msg::Float32 & msg)
{
  msg.data = 0;
  return true;
}

}  // namespace nav2_plugins

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(nav2_plugins::PublishSpinSpeedAction, "PublishSpinSpeed");
