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

#include <fstream>
#include <sstream>

class ReplanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    board = BT::Blackboard::create();
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 8.0;
    goal.pose.orientation.w = 1.0;
    board->set("goal", goal);
    factory.registerNodeType<PathGoalMatches>("PathGoalMatches");
    factory.registerSimpleAction("SelectEfficientPath", [](BT::TreeNode & node) {
      std::string path; node.getInput("candidate", path); node.setOutput("selected", path);
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("candidate"), BT::InputPort<std::string>("current"),
        BT::OutputPort<std::string>("selected")});
    factory.registerSimpleAction("ComputePathToPose", [this](BT::TreeNode & node) {
      node.setOutput("path", plan_ok ? std::string("route_A") : std::string(""));
      if (change_goal_during_plan) {
        auto changed = board->get<geometry_msgs::msg::PoseStamped>("goal");
        changed.pose.position.y += 1.0;
        board->set("goal", changed);
      }
      return plan_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }, {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::string>("planner_id"), BT::OutputPort<std::string>("path")});
    factory.registerSimpleCondition("IsPathValid", [this](BT::TreeNode & node) {
      ++validation_calls;
      std::string path;
      return node.getInput("path", path) && !path.empty() && path_valid ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }, {BT::InputPort<std::string>("path"), BT::InputPort<int>("server_timeout")});
    // Exercise the production subtree, including typed SetBlackboard copies.
    std::ifstream stream(LIDAR_TREE_PATH);
    std::stringstream buffer;
    buffer << stream.rdbuf();
    auto xml = buffer.str();
    const auto begin = xml.find("<Fallback name=\"ReplanOrKeepValid\">");
    ASSERT_NE(begin, std::string::npos);
    const auto end = xml.rfind("</Fallback>") + std::string("</Fallback>").size();
    tree = factory.createTreeFromText(
      "<root main_tree_to_execute='Test'><BehaviorTree ID='Test'>" +
      xml.substr(begin, end - begin) + "</BehaviorTree></root>", board);
  }
  bool plan_ok{true}, path_valid{true}, change_goal_during_plan{false};
  int validation_calls{0};
  BT::Blackboard::Ptr board;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
};

TEST_F(ReplanTest, FailedReplanKeepsOnlyCollisionValidatedPath)
{
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  plan_ok = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<std::string>("path"), "route_A");
  EXPECT_EQ(board->get<std::string>("candidate_path"), "");
  EXPECT_EQ(validation_calls, 1);
  path_valid = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(ReplanTest, NoFallbackBeforeFirstPlan)
{
  plan_ok = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(validation_calls, 0);
}

TEST_F(ReplanTest, NewGoalYawCannotReuseOldPath)
{
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  auto goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.orientation.z = 1.0;
  goal.pose.orientation.w = 0.0;
  board->set("goal", goal);
  plan_ok = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(validation_calls, 0);
}

TEST_F(ReplanTest, GoalChangedDuringPlanningCannotCommitOldResult)
{
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  change_goal_during_plan = true;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(validation_calls, 0);
}

TEST(PathSelection, KeepsSimilarPathAndAdoptsSubstantialRemainingImprovement)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SelectEfficientPath>("SelectEfficientPath");
  auto board = BT::Blackboard::create();
  auto path = [](std::initializer_list<std::pair<double,double>> xy) {
    nav_msgs::msg::Path p; p.header.frame_id = "map";
    for (auto point : xy) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = point.first; pose.pose.position.y = point.second;
      pose.pose.orientation.w = 1.0; p.poses.push_back(pose);
    }
    return p;
  };
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Test'><BehaviorTree ID='Test'>"
    "<SelectEfficientPath candidate='{candidate}' current='{current}' selected='{selected}'/>"
    "</BehaviorTree></root>", board);
  auto old = path({{0,0},{5,1},{10,0}});
  board->set("current", old);
  board->set("candidate", path({{0,0},{5,-1},{10,0}}));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<nav_msgs::msg::Path>("selected"), old);
  board->set("current", path({{0,0},{5,4},{10,0}}));
  auto direct = path({{0,0},{5,0},{10,0}});
  board->set("candidate", direct);
  tree.tickRoot();
  EXPECT_EQ(board->get<nav_msgs::msg::Path>("selected"), direct);
  // A geometrically short manoeuvre is not better if it introduces reversing
  // and gear changes. Forward-only route selection remains unchanged.
  old = path({{0,0},{5,4},{10,0}});
  board->set("current", old);
  board->set("candidate", path({{0,0},{-1,0},{0,0},{10,0}}));
  tree.tickRoot();
  EXPECT_EQ(board->get<nav_msgs::msg::Path>("selected"), old);
  board->set("candidate", direct);
  old = path({{-10,0},{0,0},{5,0},{10,0}});
  board->set("current", old);
  tree.tickRoot();
  EXPECT_EQ(board->get<nav_msgs::msg::Path>("selected"), direct);
}

TEST(PathSelection, ResumedRouteDropsTraveledPrefixWithoutChangingRemainingGeometry)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SelectEfficientPath>("SelectEfficientPath");
  auto board = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Test'><BehaviorTree ID='Test'>"
    "<SelectEfficientPath candidate='{candidate}' current='{current}' selected='{current}'/>"
    "</BehaviorTree></root>", board);
  nav_msgs::msg::Path old;
  old.header.frame_id = "map";
  for (int i = 0; i <= 400; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = i * .1;
    pose.pose.orientation.w = 1.;
    old.poses.push_back(pose);
  }
  nav_msgs::msg::Path remaining = old;
  remaining.poses.erase(remaining.poses.begin(), remaining.poses.begin() + 330);
  auto candidate = remaining;
  // Similar candidate should not change the retained route's shape.
  for (auto & pose : candidate.poses) {pose.pose.position.y = .05;}
  board->set("current", old);
  board->set("candidate", candidate);
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<nav_msgs::msg::Path>("current"), remaining);
  tree.haltTree(); // Recovery must not restore the path near x=0.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<nav_msgs::msg::Path>("current"), remaining);
  candidate.poses.erase(candidate.poses.begin(), candidate.poses.end() - 1);
  board->set("candidate", candidate);
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  auto selected = board->get<nav_msgs::msg::Path>("current");
  ASSERT_EQ(selected.poses.size(), 1u);
  EXPECT_EQ(selected.poses.back(), old.poses.back());
}

TEST(ObstacleYield, PausesOnceBeforeDetourButNewGoalDoesNotInheritPause)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<YieldToObstacle>("YieldToObstacle");
  bool valid = true;
  factory.registerSimpleCondition("Valid", [&](BT::TreeNode &) {
    return valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  auto board = BT::Blackboard::create();
  geometry_msgs::msg::PoseStamped goal; goal.header.frame_id = "map";
  goal.pose.orientation.w = 1.; board->set("goal", goal);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Test'><BehaviorTree ID='Test'>"
    "<YieldToObstacle goal='{goal}' path_goal='{cached}'><Valid/></YieldToObstacle>"
    "</BehaviorTree></root>", board);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS); // First goal has no old path.
  board->set("cached", goal);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  valid = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE); // Cancels FollowPath, enters Wait.
  tree.haltTree();
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS); // After recovery, permit detour.
  valid = true; tree.tickRoot(); valid = false;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE); // A new obstruction waits again.
  goal.pose.position.x = 1.; board->set("goal", goal);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS); // New goal can plan immediately.
}

TEST(OperatorPause, PreservesOuterGoalWhileControllerIsHalted)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_operator_pause");
  auto publisher = node->create_publisher<std_msgs::msg::Bool>(
    "/navigation/operator_paused", rclcpp::QoS(1).transient_local());
  auto board = BT::Blackboard::create(); board->set("node", node);
  board->set("goal", std::string("same_destination"));
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<OperatorPause>("OperatorPause");
  factory.registerNodeType<Work>("Work");
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<OperatorPause><Work/></OperatorPause></BehaviorTree></root>", board);
  auto work = dynamic_cast<Work *>(tree.nodes.at(1).get());
  ASSERT_NE(work, nullptr);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(work->starts, 1);
  auto publish = [&](bool paused) {
    std_msgs::msg::Bool msg; msg.data=paused;
    for(int i=0;i<50;++i) {
      publisher->publish(msg); std::this_thread::sleep_for(std::chrono::milliseconds(10));
      EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    }
  };
  publish(true); EXPECT_EQ(work->halts,1); EXPECT_EQ(work->starts,1);
  publish(true); EXPECT_EQ(work->halts,1); // No progress timer executes while paused.
  publish(false); EXPECT_EQ(work->starts,2);
  EXPECT_EQ(board->get<std::string>("goal"),"same_destination");
  tree.haltTree(); EXPECT_EQ(work->halts,2);
  rclcpp::shutdown();
}
