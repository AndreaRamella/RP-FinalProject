#include "robot.h"
#include <string>

#include "tf/transform_broadcaster.h"
#include <opencv2/imgproc.hpp>

using namespace std;

RobotBase::RobotBase(float radius_, World* w, const Pose& pose_)
    : WorldItem(w, pose_) {
  radius = radius_;
}

RobotBase::RobotBase(float radius_, WorldItem* p_, const Pose& pose_)
    : WorldItem(p_, pose_) {
  radius = radius_;
}

void RobotBase::draw() {
  int int_radius = radius * world->inv_res;
  int r2 = int_radius * int_radius;
  cerr << "r2 = " << r2 << endl;
  IntPoint p = world->world2grid(poseInWorld().translation);
  cerr << "origin = " << p << endl;
  for (int r = -int_radius; r <= int_radius; ++r) {
    for (int c = -int_radius; c <= int_radius; ++c) {
      IntPoint off(r, c);
      if (off * off > r2) continue;
      IntPoint p_test = p + IntPoint(r, c);
      if (world->inside(p_test)) {
        world->_display_image.at<uint8_t>(p_test.x, p_test.y) = 0;
      }
    }
  }
}

void Robot::timeTick(float dt) {
  Pose motion(tv * dt, 0, rv * dt);
  Pose next_pose = pose * motion;
  IntPoint ip = world->world2grid(next_pose.translation);

  int int_radius = radius * world->inv_res;
  if (!world->collides(ip, int_radius)) pose = next_pose;
}

void Robot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Update the robot's velocities based on the received cmd_vel message
  if(msg->linear.x < max_tv){
  tv = msg->linear.x;
  cerr << "Maximum linear velocity exceeded, set to speed limit" << endl;
  }else{
    tv = max_tv;
  }
  
  if(msg->angular.z < max_rv){
  rv = msg->angular.z;
  cerr << "Maximum angular velocity exceeded, set to speed limit" << endl;
  }else{
    rv = max_rv;
  }
}

nav_msgs::Odometry Robot::getOdometryMessage() {
  // Create and return an Odometry message with the robot's current pose and velocities
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
   
  odom_msg.child_frame_id = "robot_" + std::to_string(id) + "/odom";
  odom_msg.pose.pose.position.x = poseInWorld().translation.x;
  odom_msg.pose.pose.position.y = poseInWorld().translation.y;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(poseInWorld().theta);
  odom_msg.twist.twist.linear.x = tv;
  odom_msg.twist.twist.angular.z = rv;

  return odom_msg;
}