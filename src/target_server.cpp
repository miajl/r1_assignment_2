#include <r1_assignment_2/Target.h>
#include <ros/ros.h>

bool get_target_position(r1_assignment_2::Target::Request &req,
                         r1_assignment_2::Target::Response &res) {
  // TODO does this need to be an action client?
  ros::param::get("des_pos_x", res.target_x);
  ros::param::get("des_pos_y", res.target_y);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "target_server");
  ROS_INFO("[Target Server] Launched");
  ros::NodeHandle n;
  ros::ServiceServer service =
      n.advertiseService("/get_target", get_target_position);
  ros::spin();

  return 0;
}