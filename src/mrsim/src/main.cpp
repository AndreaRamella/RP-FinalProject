#include <sys/time.h>

#include <cmath>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "lidar.h"
#include "robot.h"
#include "simple_geometry.h"
#include "world.h"

#include <jsoncpp/json/json.h>

using namespace std;

double timeMillisec() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec * 1000 + tv.tv_usec * 1e-3;
}

// Global variables to hold robot objects and publishers
vector<Robot*> robots;
vector<ros::Publisher> odometry_publishers;
vector<ros::Subscriber> cmd_vel_subscribers;

vector<Lidar*> lidars;
vector<ros::Publisher> base_scan_publishers;

int main(int argc, char** argv) {

  if (argc < 2) {
    cerr << "Usage: rosrun mrsim mrsim_node <config_file.json>" << endl;
    return 1;
  }

  // Read the JSON configuration file
  ifstream config_file(argv[1]);
  Json::Value root;
  config_file >> root;

  ros::init(argc, argv, "my_simulator_node");
  ros::NodeHandle nh;

  // Load world map from the JSON configuration
  string map_filename = root["map"].asString();
  World w;
  w.loadFromImage(map_filename.c_str());
  
  // Load robots and lidars from JSON configuration
  const Json::Value items = root["items"];
  for (const auto& item : items) {
    // const stringstd::string type = item["type"].asString();
    if (item["type"] == "robot") {
      
      Pose robot_pose;

      robot_pose.translation.x = item["pose"][0].asFloat();
      robot_pose.translation.y = item["pose"][1].asFloat();
      robot_pose.theta = (item["pose"][2].asFloat());

      float radius = item["radius"].asFloat();

      string name = item["namespace"].asString();

      int parent_id = item["parent"].asInt();
        
      if (parent_id == -1){
        
        // Robot robot(radius, &w, robot_pose);
        Robot* robot = new Robot(radius, &w, robot_pose);
        robot->id = item["id"].asInt();
        robot->name = name;
        robot->max_rv = item["max_rv"].asFloat();
        robot->max_tv = item["max_tv"].asFloat();
        robot->tv = item["tv"].asFloat();
        robot->rv = item["rv"].asFloat();

        robots.push_back(robot);

        string cmd_vel_topic = robot->name + "/cmd_vel";
        ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 10, boost::bind(&Robot::cmdVelCallback, robot, _1));

        cmd_vel_subscribers.push_back(cmd_vel_sub);

      } else {

        WorldItem* p_robot = nullptr;

        for(int i=0; i<w.num_items; i++){
          if (w.items[i]->id == parent_id){
          p_robot = w.items[i];
          }    
        }

        if (p_robot != nullptr) {

        Robot* robot = new Robot(radius, p_robot, robot_pose);
        robot->id = item["id"].asInt();
        robot->name = name;
        robot->max_rv = item["max_rv"].asFloat();
        robot->max_tv = item["max_tv"].asFloat();
        robot->tv = item["tv"].asFloat();
        robot->rv = item["rv"].asFloat();

        robots.push_back(robot);

        string cmd_vel_topic = robot->name + "/cmd_vel";
        ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 10, boost::bind(&Robot::cmdVelCallback, robot, _1));

        cmd_vel_subscribers.push_back(cmd_vel_sub);
        
        } else {
          std::cerr << "Parent robot with id " << parent_id << " not found!" << std::endl;
        }
      }

      // Create odom publisher
      string topic_name = name +"/odom";
      odometry_publishers.push_back(nh.advertise<nav_msgs::Odometry>(topic_name, 10));
      
    }
    else if (item["type"] == "lidar") {
      float fov = item["fov"].asFloat();
      float max_range = item["max_range"].asFloat();
      int num_beams = item["num_beams"].asInt();
      Pose lidar_pose;
      lidar_pose.translation.x = item["pose"][0].asFloat();
      lidar_pose.translation.y = item["pose"][1].asFloat();
      lidar_pose.theta = item["pose"][2].asFloat();
      int parent_id = item["parent"].asInt();
       
      // Find the parent robot by id, assuming it's already created
      WorldItem* parent_robot = nullptr;
      for(int i=0; i<w.num_items; i++){
        if (w.items[i]->id == parent_id) {
          parent_robot = w.items[i];
        }
      }
  
      if (parent_robot != nullptr) {
        Lidar* lidar = new Lidar(fov, max_range, num_beams, parent_robot, lidar_pose);
        lidars.push_back(lidar);
      } else {
        std::cerr << "Parent robot with id " << parent_id << " not found!" << std::endl;
      }
      // Create base_scan publisher
      string topic_name = parent_robot->name+"/base_scan";
      base_scan_publishers.push_back(nh.advertise<sensor_msgs::LaserScan>(topic_name, 10));
    }
  }

  float delay = 0.1;
  int k=0;

  while (ros::ok()) {
    double t_start = timeMillisec();
    w.timeTick(delay);
    double t_end = timeMillisec();
    cerr << "duration: " << t_end - t_start << endl;
    cerr << "image_size: " << w.rows << " " << w.cols << endl;
    w.draw();

    // Populating and publish odometry for each robot
        for (size_t i = 0; i < robots.size(); ++i) {
            nav_msgs::Odometry odom_msg;
            odom_msg = robots[i]->getOdometryMessage();
            odometry_publishers[i].publish(odom_msg);
        }
    
    // Populating and publish base_scan for each lidar
        for (size_t i = 0; i < lidars.size(); ++i) {
            sensor_msgs::LaserScan scan;
            scan = lidars[i]->getLaserScan();
            base_scan_publishers[i].publish(scan);
        }

    k = cv::waitKeyEx(0) & 255;

    if (k == 27)  // exit on ESC
    exit(0);

    ros::spinOnce();
  }
}