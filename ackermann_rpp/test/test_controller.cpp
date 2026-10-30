#include "gtest/gtest.h"
#include "../src/controller.cpp"
TEST(AckermannCurvature, FailedDriveRequestUsesFeasibleArc) {
  EXPECT_DOUBLE_EQ(ackermann_rpp::feasibleCurvature(-0.804 / 0.25, 1.7), -1.0 / 1.7);
  EXPECT_DOUBLE_EQ(ackermann_rpp::feasibleCurvature(0.804 / 0.25, 1.7), 1.0 / 1.7);
  EXPECT_DOUBLE_EQ(ackermann_rpp::feasibleCurvature(0.12 / 0.35, 1.7), 0.12 / 0.35);
  EXPECT_THROW(ackermann_rpp::feasibleCurvature(NAN, 1.7), std::invalid_argument);
  EXPECT_THROW(ackermann_rpp::feasibleCurvature(1.0, 0.0), std::invalid_argument);
}

TEST(Reversing, DirectionChangeStopsBeforeSwitchingAndWaitsForMeasuredStop) {
  ackermann_rpp::DirectionChange gate;
  EXPECT_FALSE(gate.wait(1, 0., 0.));
  EXPECT_TRUE(gate.wait(-1, .25, 1.));
  EXPECT_TRUE(gate.wait(-1, .1, 2.));
  EXPECT_TRUE(gate.wait(-1, 0., 3.));
  EXPECT_TRUE(gate.wait(-1, 0., 3.2));
  EXPECT_FALSE(gate.wait(-1, 0., 3.4));
  EXPECT_FALSE(gate.wait(-1, -.35, 4.));
  EXPECT_TRUE(gate.wait(1, -.2, 5.));
  EXPECT_TRUE(gate.wait(1, 0., 6.));
  EXPECT_FALSE(gate.wait(1, 0., 6.4));
}

TEST(Reversing, ExplicitReversePathDiffersFromForwardPathBehindRobot) {
  nav_msgs::msg::Path path;
  for(double x : {-2., -1., 0.}) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x=x; p.pose.orientation.w=1.; path.poses.push_back(p);
  }
  EXPECT_EQ(ackermann_rpp::pathDirection(path), 1);
  std::reverse(path.poses.begin(),path.poses.end());
  EXPECT_EQ(ackermann_rpp::pathDirection(path), -1);
}

TEST(Reversing, FilledFootprintRejectsUnknownOccupiedAndMapEdges) {
  nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(auto xy : {std::pair<double,double>{.61,.335},{.61,-.335},{-.27,-.335},{-.27,.335}}) {
    geometry_msgs::msg::Point p; p.x=xy.first; p.y=xy.second; footprint.push_back(p);
  }
  EXPECT_TRUE(ackermann_rpp::knownFootprint(map,footprint,0.,0.,0.));
  map.setCost(50,50,255); // Inside, not just on the perimeter.
  EXPECT_FALSE(ackermann_rpp::knownFootprint(map,footprint,0.,0.,0.));
  map.setCost(50,50,254);
  EXPECT_FALSE(ackermann_rpp::knownFootprint(map,footprint,0.,0.,0.));
  map.setCost(50,50,150); // Soft inflation isn't an additional hard margin.
  EXPECT_TRUE(ackermann_rpp::knownFootprint(map,footprint,0.,0.,0.));
  EXPECT_FALSE(ackermann_rpp::knownFootprint(map,footprint,-2.4,0.,0.));
}
