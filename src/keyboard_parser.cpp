#include <ros/ros.h>

#include <iostream>
#include <string>

#include "std_msgs/String.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_parser");
  ROS_INFO("[Keyboard Parser] Launched");
  ros::NodeHandle n;
  ros::Publisher key_pub = n.advertise<std_msgs::String>("/key_in", 1000);

  std_msgs::String input_line;
  while (ros::ok()) {
    getline(std::cin, input_line.data);
    key_pub.publish(input_line);
  }

  // exit
  return 0;
}