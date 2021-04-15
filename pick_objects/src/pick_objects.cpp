#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //
  ros::Publisher status_pub = n.advertise<std_msgs::UInt8>("/robot_status", 1);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  std_msgs::UInt8 status_msg;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 4;
  goal.target_pose.pose.position.y = -11;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the base reached the pickup zone");

    status_msg.data = 1; // pickup reached
    status_pub.publish(status_msg);
  }
  else
    ROS_INFO("The base failed to reach the pickup zone");

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting 5 sec for picking up");
  }

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 5;
  goal.target_pose.pose.position.y = -19;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the base reached the drop-off zone");
    
    status_msg.data = 2; // dropoff reached
    status_pub.publish(status_msg);
  }
  else
    ROS_INFO("The base failed to reach the drop-off zone");

  ros::Duration(5.0).sleep();

  return 0;
}
