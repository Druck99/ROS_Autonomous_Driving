#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // //we'll send a goal to the robot to move 1 meter forward
  // goal.target_pose.header.frame_id = "OurCar/INS";
  // goal.target_pose.header.stamp = ros::Time::now();

  // goal.target_pose.pose.position.x = 100.0;
  // goal.target_pose.pose.orientation.w = 1.0;

  // 我们将发送一个目标到 world 坐标系下 (200, 50)
  goal.target_pose.header.frame_id = "world";
  goal.target_pose.header.stamp    = ros::Time::now();

  goal.target_pose.pose.position.x = 195.0;
  goal.target_pose.pose.position.y =  50.0;
  goal.target_pose.pose.position.z =   0.0;  // 如需指定高度，可修改此行
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;  // 保持朝向不变，指向世界坐标系的零旋转

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}