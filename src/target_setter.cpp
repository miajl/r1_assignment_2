#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <r1_assignment_2/PlanningAction.h>
#include <r1_assignment_2/PosVel.h>
#include <ros/ros.h>

#include <memory>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

// pointer for action client
std::unique_ptr<actionlib::SimpleActionClient<r1_assignment_2::PlanningAction>>
    ac;
// posvel publisher
ros::Publisher posvel_pub;

std::string prev_state = "";

r1_assignment_2::PlanningGoal goal;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const r1_assignment_2::PlanningResultConstPtr& result) {
  ROS_INFO("[TARGET SETTER] Finished in state %s", state.toString().c_str());
  ROS_INFO(
      "[TARGET SETTER] Select a destination in rviz using clicked point or "
      "type \"T:[x_pos],[y_pos]\" + Enter");
}

// Called once when the goal becomes active
void activeCb() { ROS_INFO("[TARGET SETTER] Goal just went active"); }

void feedbackCb(const r1_assignment_2::PlanningFeedbackConstPtr& feedback) {
  // only print feedback if state has changed to avoid spamming the konsole
  // all of the state messages have different length so this comparison workds
  if (feedback->stat.length() != prev_state.length()) {
    ROS_INFO("[TARGET SETTER] State: %s X: %f Y: %f", feedback->stat.c_str(),
             feedback->actual_pose.position.x,
             feedback->actual_pose.position.y);
    prev_state = feedback->stat;
  }
}

void clicked_point_callback(const geometry_msgs::PointStamped msg) {
  ac->cancelAllGoals();
  ROS_INFO("[TARGET SETTER] User selected a point at %lf %lf", msg.point.x,
           msg.point.y);
  goal.target_pose.pose.position.x = msg.point.x;
  goal.target_pose.pose.position.y = msg.point.y;
  ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  ROS_INFO("[TARGET SETTER] Sent goal, type 'x' + Enter to cancel");
}

void odom_callback(const nav_msgs::Odometry msg) {
  r1_assignment_2::PosVel pv;
  pv.x = msg.pose.pose.position.x;
  pv.y = msg.pose.pose.position.y;
  pv.vel_x = msg.twist.twist.linear.x;
  pv.vel_z = msg.twist.twist.angular.z;
  posvel_pub.publish(pv);
}

void key_callback(const std_msgs::String msg) {
  const char* input = msg.data.c_str();
  if (input[0] == 'x' || input[0] == 'X') {
    ac->cancelAllGoals();
    ROS_INFO("[TARGET SETTER] Cancelled Goal");
  } else if (input[0] == 'T') {
    sscanf(input, "T:%lf,%lf", &goal.target_pose.pose.position.x,
           &goal.target_pose.pose.position.y);
    ac->cancelAllGoals();
    ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    ROS_INFO("x %lf", goal.target_pose.pose.position.x);
    ROS_INFO("y %lf", goal.target_pose.pose.position.y);
    ROS_INFO("[TARGET SETTER] Sent goal, type 'x' + Enter to cancel");
  } else {
    ROS_INFO("Bad input");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_setter");
  ROS_INFO("[Target Setter] Launched");
  // create the action client
  ac = std::make_unique<
      actionlib::SimpleActionClient<r1_assignment_2::PlanningAction>>(
      "/reaching_goal", true);
  // true causes the client to spin its own thread

  ros::NodeHandle n;
  ros::Subscriber clicked_point_sub =
      n.subscribe("/clicked_point", 1000, clicked_point_callback);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odom_callback);
  ros::Subscriber key_sub = n.subscribe("/key_in", 1000, key_callback);
  posvel_pub = n.advertise<r1_assignment_2::PosVel>("/posvel", 1000);

  ROS_INFO("[TARGET SETTER] Waiting for action server to start.");
  // wait for the action server to start
  ac->waitForServer();  // will wait for infinite time

  ROS_INFO(
      "[TARGET SETTER] Select a destination in rviz using clicked point or "
      "type \"T:[x_pos],[y_pos]\" + Enter");

  ros::spin();

  // exit
  return 0;
}