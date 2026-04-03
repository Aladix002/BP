// BT plugin pre Nav2: Action klient na /follow_ball (waverower FollowBall.action)
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "waverower/action/follow_ball.hpp"

namespace waverower_behavior_tree
{

class FollowBallBtAction
  : public nav2_behavior_tree::BtActionNode<waverower::action::FollowBall>
{
  using Action = waverower::action::FollowBall;

public:
  FollowBallBtAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<Action>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick() override
  {
    std::string color;
    if (!getInput("ball_color", color)) {
      goal_.ball_color = "";
    } else {
      goal_.ball_color = color;
    }
    double dur = 0.0;
    if (!getInput("max_duration_sec", dur)) {
      goal_.max_duration_sec = 0.0;
    } else {
      goal_.max_duration_sec = dur;
    }
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("ball_color", "", "Farba alebo prázdne = default servera"),
      BT::InputPort<double>("max_duration_sec", 0.0, "0 = bez limitu (s)"),
    });
  }

  BT::NodeStatus on_success() override
  {
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_aborted() override
  {
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_cancelled() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace waverower_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<waverower_behavior_tree::FollowBallBtAction>(
        name, "follow_ball", config);
    };
  factory.registerBuilder<waverower_behavior_tree::FollowBallBtAction>(
    "FollowBall", builder);
}
