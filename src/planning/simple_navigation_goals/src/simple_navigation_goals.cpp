#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <ros/package.h>
#include <cmath> // 用于计算距离
#include <nav_msgs/Odometry.h> // Odometry 消息定义


// 定义目标点结构体
struct GoalPoint {
    double x;
    double y;
};

// 读取CSV文件中的目标点
std::vector<GoalPoint> readGoalPoints(const std::string& filename) {
    std::vector<GoalPoint> points;
    std::ifstream file(filename.c_str());
    
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return points;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        GoalPoint point;
        
        try {
            // 读取X坐标
            std::getline(ss, token, ',');
            point.x = std::stod(token);
            
            // 读取Y坐标
            std::getline(ss, token, ',');
            point.y = std::stod(token);
            
            points.push_back(point);
        } catch (...) {
            ROS_WARN("Skipping invalid line: %s", line.c_str());
        }
    }
    
    file.close();
    return points;
}

// 全局变量存储当前位置
geometry_msgs::Point current_position;
bool position_updated = false;

// 位置回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position = msg->pose.pose.position;
    position_updated = true;
}

// 计算两点间距离
double distanceBetweenPoints(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx + dy*dy);
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  //tell the action client that we want to spin a thread by default. ROS 回调（spin thread）
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // 从CSV文件读取目标点
  // 在main函数中获取包路径
  std::string package_path = ros::package::getPath("simple_navigation_goals");
  std::string csv_path = package_path + "/src/goals.csv"; // 假设csv在src目录下
  std::vector<GoalPoint> goals = readGoalPoints(csv_path);  // 替换为实际路径
  
  if (goals.empty()) {
    ROS_ERROR("No valid goals loaded. Exiting.");
    return 1;
  }

  // 订阅odom话题获取当前位置
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
  
  // 等待首次位置更新
  ros::Rate rate(10); // 10Hz
  while (ros::ok() && !position_updated) {
      ros::spinOnce();
      rate.sleep();
  }
  ROS_INFO("Initial position received!");

  // 设置目标点参数
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "world";
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  // 遍历所有目标点
  for (size_t i = 0; i < goals.size(); ++i) {
      const GoalPoint& target = goals[i];
      
      // 设置目标位置
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = target.x;
      goal.target_pose.pose.position.y = target.y;
      
      ROS_INFO("Sending goal %zu: (%.2f, %.2f)", i+1, target.x, target.y);
      ac.sendGoal(goal);
      
      // 等待机器人接近目标点（1米范围内）
      const double proximity_threshold = 3.0; // 1米
      bool goal_reached = false;
      bool action_completed = false;
      
      while (ros::ok() && !goal_reached) {
        ros::spinOnce(); // 处理回调
        
        // // 检查动作状态
        // actionlib::SimpleClientGoalState state = ac.getState();
        // action_completed = state.isDone();
        
        // 检查距离
        geometry_msgs::Point target_point;
        target_point.x = target.x;
        target_point.y = target.y;
        target_point.z = 0.0;
        
        double distance = distanceBetweenPoints(current_position, target_point);
        
        if (distance <= proximity_threshold) {
            ROS_INFO("Goal %zu reached! Distance: %.2f meters", i+1, distance);
            goal_reached = true;
        }
        
        // // 超时检查（300秒）
        // if ((ros::Time::now() - start_time).toSec() > 300.0) {
        //     ROS_WARN("Timeout for goal %zu, moving to next", i+1);
        //     break;
        // }
        
        // // 显示状态
        // ROS_INFO_THROTTLE(5, "Goal %zu - Distance: %.2fm, State: %s", 
        //                   i+1, distance, state.toString().c_str());
        
        rate.sleep();
      }

      // if (reached) {
      //     if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      //         ROS_INFO("Reached goal %zu!", i+1);
      //     } else {
      //         ROS_WARN("Failed to reach goal %zu. Error: %s", 
      //                 i+1, ac.getState().toString().c_str());
      //         // 可选：重试逻辑
      //         i--; // 重试当前点
      //     }
      // } else {
      //     ROS_ERROR("Action server not responding!");
      //     return 1;
      // }
  }

  ROS_INFO("All goals completed!");
  return 0;
}