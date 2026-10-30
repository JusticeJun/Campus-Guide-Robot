// Copyright 2026 Campus Guide Robot contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include "../src/localization_ready.cpp"

class Work : public BT::StatefulActionNode
{
public:
  Work(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {++starts; return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
  void onHalted() override {++halts;}
  int starts{0};
  int halts{0};
};

TEST(LocalizationReady, HaltsSubtreeKeepsOuterGoalAndRestartsWork)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_localization_pause");
  auto publisher = node->create_publisher<std_msgs::msg::Bool>("/navigation/ready", 1);
  auto board = BT::Blackboard::create();
  board->set("node", node);
  board->set("goal", std::string("original_destination"));
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<LocalizationReady>("LocalizationReady");
  factory.registerNodeType<Work>("Work");
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<LocalizationReady><Work/></LocalizationReady></BehaviorTree></root>", board);
  auto work = dynamic_cast<Work *>(tree.nodes.at(1).get());
  ASSERT_NE(work, nullptr);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(work->starts, 0);
  auto publish = [&](bool ready) {
      std_msgs::msg::Bool msg;
      msg.data = ready;
      // Allow local DDS discovery/delivery while repeatedly exercising the real tree.
      for (int i = 0; i < 50; ++i) {
        publisher->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
      }
    };
  publish(true);
  EXPECT_EQ(work->starts, 1);
  publish(false);
  EXPECT_EQ(work->halts, 1);
  EXPECT_EQ(work->starts, 1);
  publish(true);
  EXPECT_EQ(work->starts, 2);
  EXPECT_EQ(board->get<std::string>("goal"), "original_destination");
  tree.haltTree();  // Outer cancellation still halts child work normally.
  EXPECT_EQ(work->halts, 2);
  rclcpp::shutdown();
}
