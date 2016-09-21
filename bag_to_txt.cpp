#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>
#include <fstream>
#include <memory>
#include <iostream>

#include <string>
#include <sstream>

std::ofstream scan_file("base_scan.txt");
std::ofstream odom_file("odom.txt");

void from_file_to_scan(std::ifstream& scan_file, std::ifstream& odom_file, TransformedLaserScan& tls) {
  std::string scan_line;
  std::string odom_line;
  getline(scan_file,scan_line);
  getline(odom_file,odom_line);
  std::stringstream scan_stream(scan_line);
  std::stringstream odom_stream(odom_line);
  double angle_min, angle_max, angle_inc;
  scan_stream >> angle_min >> angle_max >> angle_inc;
  double range;
  int i = 0;
  while (scan_stream >> range) {
    tls.points.push_bask({range, angle_min+angle_inc*i});
    i++;
  }
  odom_stream >> tls.d_x >> tls.d_y >> tls.d_yaw;
}

void base_scan_getter(const sensor_msgs::LaserScan& msg) {
  scan_file << msg.angle_min << " " << msg.angle_max << " " << msg.angle_increment;
  for (auto& point : msg.ranges) {
    scan_file << " " << point;
  }
  scan_file << std::endl;
  return;
}


void odom_getter(const nav_msgs::Odometry& msg) {
  static double prev_x, prev_y, prev_theta;
  static int count = 0;
  if (count == 0) {
    prev_x = msg.pose.pose.position.x;
    prev_y = msg.pose.pose.position.y;
    prev_theta = tf::getYaw(msg.pose.pose.orientation);
    count = 1;
    return;
  }
  double current_x,current_y,current_theta;
  current_x = msg.pose.pose.position.x;
  current_y = msg.pose.pose.position.y;
  current_theta = tf::getYaw(msg.pose.pose.orientation);
  odom_file << current_x - prev_x << " " << current_y - prev_y << " " << current_theta - prev_theta << std::endl;
  prev_x = current_x;
  prev_y = current_y;
  prev_theta = current_theta;
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "converter");
  ros::NodeHandle nh;
  ros::Subscriber sub_scan = nh.subscribe("/base_scan",1000,base_scan_getter);
  ros::Subscriber sub_odom = nh.subscribe("/base_odometry/odom",1000,odom_getter);
  ros::Rate r(1);
  while (ros::ok())
    ros::spin();
  scan_file.close();
  odom_file.close();
  return 0;
}
