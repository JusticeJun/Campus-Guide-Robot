// Copyright 2026 Campus Guide Robot contributors
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// Keep NavigateToPose RUNNING while halting its planner/controller subtree.
// Restarting the subtree forces ComputePathToPose before a fresh FollowPath.
class LocalizationReady : public BT::DecoratorNode
{
public:
  LocalizationReady(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/navigation/ready", rclcpp::QoS(1),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        ready_ = msg->data;
      }, options);
  }

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    executor_.spin_some();
    if (!ready_) {
      resetChild();  // Nav2 BtActionNode::halt cancels the subordinate action.
      return BT::NodeStatus::RUNNING;
    }
    return child_node_->executeTick();
  }

private:
  bool ready_{false};
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LocalizationReady>("LocalizationReady");
}
