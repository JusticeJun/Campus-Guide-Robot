// Copyright 2026 Campus Guide Robot contributors
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <cmath>
#include <limits>
#include "nav_msgs/msg/path.hpp"
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

// Pause only subordinate planning/control actions; the NavigateToPose goal and
// blackboard survive. A halted FollowPath does not run its progress timeout.
class OperatorPause : public BT::DecoratorNode
{
public:
  OperatorPause(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/navigation/operator_paused", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {paused_ = msg->data;}, options);
  }
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    executor_.spin_some();
    if (paused_) {
      resetChild();
      return BT::NodeStatus::RUNNING;
    }
    return child_node_->executeTick();
  }
private:
  bool paused_{false};
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

// Never reuse a path belonging to a previous goal (including a changed yaw).
// Ignore stamps: a republished identical destination is still the same goal.
class PathGoalMatches : public BT::ConditionNode
{
public:
  PathGoalMatches(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("path_goal")};
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal, path_goal;
    if (!getInput("goal", goal) || !getInput("path_goal", path_goal)) {
      return BT::NodeStatus::FAILURE;
    }
    return goal.header.frame_id == path_goal.header.frame_id && goal.pose == path_goal.pose ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// Prefer a materially shorter valid route, not a different equal-cost side on
// every scan. The XML verifies the old path against the current costmap first.
class SelectEfficientPath : public BT::SyncActionNode
{
public:
  SelectEfficientPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("candidate"),
      BT::InputPort<nav_msgs::msg::Path>("current"),
      BT::OutputPort<nav_msgs::msg::Path>("selected")};
  }
  BT::NodeStatus tick() override {
    nav_msgs::msg::Path candidate, current;
    if (!getInput("candidate", candidate) || candidate.poses.empty() ||
      !getInput("current", current) || current.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    auto distance = [](const auto & a, const auto & b) {
      return std::hypot(a.pose.position.x - b.pose.position.x,
                       a.pose.position.y - b.pose.position.y);
    };
    size_t nearest = 0;
    double gap = std::numeric_limits<double>::infinity();
    size_t cusp=current.poses.size();
    for(size_t i=1;i+1<current.poses.size();++i) {
      const auto & a=current.poses[i-1].pose.position;
      const auto & b=current.poses[i].pose.position;
      const auto & c=current.poses[i+1].pose.position;
      if((b.x-a.x)*(c.x-b.x)+(b.y-a.y)*(c.y-b.y)<-1e-8) {cusp=i;break;}
    }
    for (size_t i = 0; i < current.poses.size(); ++i) {
      if(i>cusp) {break;} // Preserve unexecuted reverse legs at overlapping coordinates.
      const double d = distance(candidate.poses.front(), current.poses[i]);
      if (d < gap-1e-6) {gap = d; nearest = i;}
    }
    if(nearest==cusp && cusp>0 && gap>.08) {--nearest;}
    // Preserve forward-only scoring. Prefer fewer gear changes and less
    // reversing when comparing valid routes, instead of shortest length alone.
    auto remaining_cost = [&](const nav_msgs::msg::Path & p, size_t start) {
      double cost=0.; int previous=0;
      for(size_t i=start+1;i<p.poses.size();++i) {
        const auto & a=p.poses[i-1].pose; const auto & b=p.poses[i].pose;
        const double length=distance(p.poses[i-1],p.poses[i]);
        if(length<1e-5) {continue;}
        const auto & q=a.orientation;
        const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        const int gear=(b.position.x-a.position.x)*std::cos(yaw)+
          (b.position.y-a.position.y)*std::sin(yaw)<0 ? -1 : 1;
        cost+=length*(gear<0 ? 3. : 1.);
        if(previous && previous!=gear) {cost+=1.;}
        previous=gear;
      }
      return cost;
    };
    const double old_length=gap+remaining_cost(current,nearest);
    const double new_length=remaining_cost(candidate,0);
    // Rejoin after a significant tracking offset; otherwise require both a
    // 1 m and a 10% improvement of the REMAINING path, not the original path.
    auto side = [&](const nav_msgs::msg::Path & p, size_t start) {
      double walked = 0.0;
      size_t i = start;
      for (; i + 1 < p.poses.size() && walked < 3.0; ++i) {
        walked += distance(p.poses[i], p.poses[i + 1]);
      }
      const auto & origin = candidate.poses.front().pose;
      const auto & q = origin.orientation;
      const double yaw = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
      const double dx = p.poses[i].pose.position.x - origin.position.x;
      const double dy = p.poses[i].pose.position.y - origin.position.y;
      return -std::sin(yaw)*dx + std::cos(yaw)*dy;
    };
    const double old_side = side(current, nearest), new_side = side(candidate, 0);
    const bool switches_side = old_side * new_side < 0 &&
      std::abs(old_side) > 0.3 && std::abs(new_side) > 0.3;
    const double absolute_gain = switches_side ? 2.0 : 1.0;
    const double relative_gain = switches_side ? 0.80 : 0.90;
    const bool replace = current.header.frame_id != candidate.header.frame_id ||
      gap > 0.6 || (new_length + absolute_gain < old_length && new_length < relative_gain * old_length);
    if (replace) {
      setOutput("selected", candidate);
    } else {
      // RPP prunes its internal path while driving, but a restarted FollowPath
      // gets this blackboard copy again. Do not reintroduce the departed prefix:
      // RPP searches for the nearest pose only within a bounded initial length.
      // Keep the remaining geometry (and goal orientation) exactly unchanged.
      current.poses.erase(current.poses.begin(), current.poses.begin() + nearest);
      setOutput("selected", current);
    }
    return BT::NodeStatus::SUCCESS;
  }
};

// A newly blocked existing route first halts FollowPath through RecoveryNode.
// Its Wait action gives the obstacle time to leave before detour computation.
class YieldToObstacle : public BT::DecoratorNode {
public:
 YieldToObstacle(const std::string & name, const BT::NodeConfiguration & config)
 : BT::DecoratorNode(name, config) {}
 static BT::PortsList providedPorts() {return PathGoalMatches::providedPorts();}
 BT::NodeStatus tick() override {
   geometry_msgs::msg::PoseStamped goal, cached;
   if (!getInput("goal", goal) || !getInput("path_goal", cached) ||
     goal.header.frame_id != cached.header.frame_id || goal.pose != cached.pose) {
     paused_ = false; return BT::NodeStatus::SUCCESS;
   }
   const auto valid = child_node_->executeTick();
   if (valid == BT::NodeStatus::RUNNING) {return valid;}
   resetChild();
   if (valid == BT::NodeStatus::SUCCESS) {paused_ = false; return valid;}
   if (!paused_) {paused_ = true; return BT::NodeStatus::FAILURE;}
   return BT::NodeStatus::SUCCESS;
 }
private:
 bool paused_{false};
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LocalizationReady>("LocalizationReady");
  factory.registerNodeType<OperatorPause>("OperatorPause");
  factory.registerNodeType<PathGoalMatches>("PathGoalMatches");
  factory.registerNodeType<SelectEfficientPath>("SelectEfficientPath");
  factory.registerNodeType<YieldToObstacle>("YieldToObstacle");
}
