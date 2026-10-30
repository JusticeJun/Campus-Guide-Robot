// Copyright 2026 Campus Guide Robot contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/utils.h"

namespace ackermann_rpp {
inline size_t firstCusp(const nav_msgs::msg::Path & path) {
  for(size_t i=1;i+1<path.poses.size();++i) {
    const auto & a=path.poses[i-1].pose.position;
    const auto & b=path.poses[i].pose.position;
    const auto & c=path.poses[i+1].pose.position;
    if((b.x-a.x)*(c.x-b.x)+(b.y-a.y)*(c.y-b.y)<-1e-8) {return i;}
  }
  return path.poses.size();
}
// Path orientation is vehicle heading, including on reverse segments.
// A forward path behind an off-track rover is NOT permission to reverse.
inline int pathDirection(const nav_msgs::msg::Path & path) {
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i-1].pose;
    const auto & b = path.poses[i].pose;
    const double dx = b.position.x - a.position.x, dy = b.position.y - a.position.y;
    if (std::hypot(dx, dy) < 1e-5) {continue;}
    const double yaw = tf2::getYaw(a.orientation);
    return dx * std::cos(yaw) + dy * std::sin(yaw) < 0 ? -1 : 1;
  }
  return 0;
}

class DirectionChange {
public:
  bool wait(int wanted, double measured, double now) {
    if (!std::isfinite(measured)) {stopped_at_ = -1.; return true;}
    const bool changing = direction_ != 0 && wanted != direction_;
    const bool opposed = wanted * measured < -0.03;
    if (!changing && !opposed) {direction_ = wanted; stopped_at_ = -1.; return false;}
    if (std::abs(measured) > 0.03 || !std::isfinite(measured)) {
      stopped_at_ = -1.; return true;
    }
    if (stopped_at_ < 0.) {stopped_at_ = now;}
    if (now - stopped_at_ < 0.3) {return true;}
    direction_ = wanted; stopped_at_ = -1.; return false;
  }
private:
  int direction_{0};
  double stopped_at_{-1.};
};

// The rover footprint is rectangular. Check its filled padded envelope, not
// only the perimeter (which could enclose an unknown/occupied cell unnoticed).
inline bool knownFootprint(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double x, double y, double yaw)
{
  if (footprint.empty()) {return false;}
  double xmin=INFINITY, xmax=-INFINITY, ymin=INFINITY, ymax=-INFINITY;
  for (const auto & p : footprint) {
    xmin=std::min(xmin,p.x); xmax=std::max(xmax,p.x);
    ymin=std::min(ymin,p.y); ymax=std::max(ymax,p.y);
  }
  const double margin=map.getResolution()/std::sqrt(2.);
  const double radius=std::hypot(std::max(std::abs(xmin),std::abs(xmax)),
    std::max(std::abs(ymin),std::abs(ymax)))+margin;
  const int left=std::floor((x-radius-map.getOriginX())/map.getResolution());
  const int right=std::floor((x+radius-map.getOriginX())/map.getResolution());
  const int bottom=std::floor((y-radius-map.getOriginY())/map.getResolution());
  const int top=std::floor((y+radius-map.getOriginY())/map.getResolution());
  if (left<0 || bottom<0 || right>=int(map.getSizeInCellsX()) || top>=int(map.getSizeInCellsY())) {
    return false;
  }
  for (int j=bottom;j<=top;++j) {for(int i=left;i<=right;++i) {
    double wx,wy; map.mapToWorld(i,j,wx,wy);
    const double bx=(wx-x)*std::cos(yaw)+(wy-y)*std::sin(yaw);
    const double by=-(wx-x)*std::sin(yaw)+(wy-y)*std::cos(yaw);
    if (bx>=xmin-margin && bx<=xmax+margin && by>=ymin-margin && by<=ymax+margin &&
      map.getCost(i,j)>=nav2_costmap_2d::LETHAL_OBSTACLE) {return false;}
  }}
  return true;
}
}
