#include <math.h>
#include <r1_assignment_2/DistanceSpeed.h>
#include <r1_assignment_2/PosVel.h>
#include <r1_assignment_2/Target.h>
#include <ros/ros.h>

float distance;
float average_speed;
int avg_window_size;
std::list<float> stored_speeds;
ros::ServiceClient target_client;
float target_x;
float target_y;

bool get_distance_speed(r1_assignment_2::DistanceSpeed::Request &req,
                        r1_assignment_2::DistanceSpeed::Response &res) {
  res.distance = distance;
  res.average_speed = average_speed;
  return true;
}

void posvel_callback(const r1_assignment_2::PosVel msg) {
  r1_assignment_2::Target target;

  if (target_client.call(target)) {
    target_x = target.response.target_x;
    target_y = target.response.target_y;
  } else {
    ROS_ERROR("Failed to call service get_target");
    return;
  }
  // TODO replace with service
  ros::param::get("des_pos_x", target_x);
  ros::param::get("des_pos_y", target_y);

  distance = sqrt(pow(target_x - msg.x, 2) + pow(target_y - msg.y, 2));
  ros::param::get("avg_window_size", avg_window_size);

  stored_speeds.push_back(msg.vel_x);

  if (stored_speeds.size() > avg_window_size) {
    float popped_speed = stored_speeds.front();
    stored_speeds.pop_front();
    average_speed -= popped_speed / avg_window_size;
    average_speed += msg.vel_x / avg_window_size;
  } else {
    average_speed = 0;

    for (const auto speed : stored_speeds) {
      average_speed += speed / stored_speeds.size();
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "distance_speed_server");
  ROS_INFO("[Distance Speed Server] Launched");
  ros::NodeHandle n;
  ros::Subscriber posvel_sub = n.subscribe("/posvel", 1000, posvel_callback);
  ros::ServiceServer service =
      n.advertiseService("/get_distance_speed", get_distance_speed);
  target_client = n.serviceClient<r1_assignment_2::Target>("/get_target");
  target_client.waitForExistence();
  ros::spin();

  return 0;
}