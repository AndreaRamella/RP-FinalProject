#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "world.h"

class RobotBase : public WorldItem {
 public:
  RobotBase(float radius_, World* w, const Pose& pose_ = Pose());

  RobotBase(float radius_, WorldItem* p_, const Pose& pose_ = Pose());

  void draw() override;

  float radius;
};

class Robot : public RobotBase {
 public:
  Robot(float radius_, World* w, const Pose& pose_ = Pose())
      : RobotBase(radius_, w, pose_) {}

  Robot(float radius_, WorldItem* p_, const Pose& pose_ = Pose())
      : RobotBase(radius_, p_, pose_) {}

  void timeTick(float dt);

  float tv = 0, rv = 0;
  float max_rv = 0, max_tv = 0;

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  nav_msgs::Odometry getOdometryMessage();

 private:
  ros::NodeHandle nh;
  ros::Publisher odometry_pub;
  ros::Subscriber cmd_vel_sub;
  geometry_msgs::Twist current_vel;
};