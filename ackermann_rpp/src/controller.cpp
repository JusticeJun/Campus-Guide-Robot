// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// computeVelocityCommands adapted from Navigation2 humble RPP (Apache-2.0).
// Keep native cost regulation/TF/collision checks; constrain curvature BEFORE them.
#include <cmath>
#include <stdexcept>
#include <chrono>
#include "reverse_policy.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
namespace ackermann_rpp {
double feasibleCurvature(double curvature, double radius) {
  if (!std::isfinite(curvature) || !std::isfinite(radius) || radius <= 0.0) {
    throw std::invalid_argument("Invalid Ackermann curvature/radius");
  }
  return std::clamp(curvature, -1.0 / radius, 1.0 / radius);
}
class Controller : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
   RegulatedPurePursuitController::configure(parent, name, tf, map);
   auto node = parent.lock();
   nav2_util::declare_parameter_if_not_declared(node, name + ".minimum_turning_radius", rclcpp::ParameterValue(1.7));
   node->get_parameter(name + ".minimum_turning_radius", minimum_radius_);
   feasibleCurvature(0.0, minimum_radius_);
   nav2_util::declare_parameter_if_not_declared(node, name + ".maximum_reverse_speed", rclcpp::ParameterValue(0.4));
   node->get_parameter(name + ".maximum_reverse_speed", maximum_reverse_speed_);
   if (!std::isfinite(maximum_reverse_speed_) || maximum_reverse_speed_ <= 0. || maximum_reverse_speed_ > 0.4) {
     throw std::invalid_argument("maximum_reverse_speed must be in (0, 0.4]");
   }
   if (use_rotate_to_heading_ || !use_collision_detection_) {
     throw std::invalid_argument("Ackermann RPP requires no in-place turns and active collision checks");
   }
 }
 geometry_msgs::msg::TwistStamped computeVelocityCommands(
   const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
   nav2_core::GoalChecker *) override;
private:
 nav_msgs::msg::Path transformOrderedPlan(const geometry_msgs::msg::PoseStamped & pose);
 double minimum_radius_{1.7};
 double maximum_reverse_speed_{0.4};
 DirectionChange direction_change_;
};

nav_msgs::msg::Path Controller::transformOrderedPlan(const geometry_msgs::msg::PoseStamped & pose)
{
  if(global_plan_.poses.empty()) {throw nav2_core::PlannerException("Received empty path");}
  geometry_msgs::msg::PoseStamped robot;
  if(!transformPose(global_plan_.header.frame_id,pose,robot)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into path frame");
  }
  auto distance=[](const auto & a,const auto & b) {return std::hypot(a.x-b.x,a.y-b.y);};
  const size_t cusp=firstCusp(global_plan_);
  size_t nearest=0;
  double best=INFINITY, walked=0.;
  // Search only the current gear segment. Otherwise a later overlapping
  // forward leg can be marginally closer and silently erase the reverse leg.
  for(size_t i=0;i<global_plan_.poses.size() && i<=cusp;++i) {
    if(i) {walked+=distance(global_plan_.poses[i-1].pose.position,global_plan_.poses[i].pose.position);}
    if(walked>max_robot_pose_search_dist_) {break;}
    const double d=distance(robot.pose.position,global_plan_.poses[i].pose.position);
    if(d<best-1e-6) {best=d;nearest=i;}
  }
  if(nearest==cusp && cusp>0 && best>.08) {--nearest;}
  global_plan_.poses.erase(global_plan_.poses.begin(),global_plan_.poses.begin()+nearest);
  nav_msgs::msg::Path local;
  local.header.frame_id=costmap_ros_->getBaseFrameID(); local.header.stamp=pose.header.stamp;
  for(const auto & waypoint:global_plan_.poses) {
    if(distance(waypoint.pose.position,robot.pose.position)>getCostmapMaxExtent()) {break;}
    geometry_msgs::msg::PoseStamped in=waypoint,out;
    in.header.frame_id=global_plan_.header.frame_id;in.header.stamp=robot.header.stamp;
    if(!transformPose(local.header.frame_id,in,out)) {
      throw nav2_core::PlannerException("Unable to transform path waypoint");
    }
    out.pose.position.z=0.;local.poses.push_back(out);
  }
  global_path_pub_->publish(local);
  if(local.poses.empty()) {throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");}
  return local;
}
geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  if (use_rotate_to_heading_ || !use_collision_detection_) {
    throw nav2_core::PlannerException("Ackermann RPP requires no in-place turns and active collision checks");
  }

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = allow_reversing_ ? transformOrderedPlan(pose) : transformGlobalPlan(pose);
  const auto & first = transformed_plan.poses.front().pose.position;
  if (std::hypot(first.x, first.y) > 0.75) {
    throw nav2_core::PlannerException("Retained path is disconnected from the rover; replan required");
  }

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  curvature = feasibleCurvature(curvature, minimum_radius_);

  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
    const int planned_direction = pathDirection(transformed_plan);
    if (planned_direction != 0 && sign != planned_direction) {
      throw nav2_core::PlannerException("Pursuit direction disagrees with planned gear; replan required");
    }
  }

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      curvature, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);

    if (linear_vel < 0.) {linear_vel = std::max(linear_vel, -maximum_reverse_speed_);}

    // Stop through an actual direction change; don't ask the ESC to reverse
    // while odometry still reports motion in the previous direction.
    const double steady_now = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (direction_change_.wait(sign < 0 ? -1 : 1, speed.linear.x, steady_now)) {
      linear_vel = 0.;
    }

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * curvature;
  }

  if (linear_vel < 0.) {
    // Rear is blind: only use known costmap space. Check the filled swept
    // footprint for at least 0.5 m and through the native collision horizon.
    double x=pose.pose.position.x, y=pose.pose.position.y, yaw=tf2::getYaw(pose.pose.orientation);
    const double distance=std::max(0.5, std::abs(linear_vel)*max_allowed_time_to_collision_up_to_carrot_);
    const int steps=std::max(1, int(std::ceil(distance/(costmap->getResolution()/2.))));
    const double dt=distance/(std::abs(linear_vel)*steps);
    for(int i=0;i<=steps;++i) {
      if (!knownFootprint(*costmap, costmap_ros_->getRobotFootprint(), x,y,yaw)) {
        throw nav2_core::PlannerException("Reverse swept footprint contains occupied or unknown costmap space");
      }
      x+=linear_vel*std::cos(yaw+angular_vel*dt/2.)*dt;
      y+=linear_vel*std::sin(yaw+angular_vel*dt/2.)*dt;
      yaw+=angular_vel*dt;
    }
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
    throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

}
PLUGINLIB_EXPORT_CLASS(ackermann_rpp::Controller, nav2_core::Controller)
