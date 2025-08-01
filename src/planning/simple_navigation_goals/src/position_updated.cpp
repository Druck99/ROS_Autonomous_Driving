#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <ros/package.h>
#include <cmath> // 用于计算距离

struct GoalPoint {
    double x;
    double y;
};

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_goal_navigation");
    ros::NodeHandle nh;
    
    // 创建move_base客户端
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    
    // 订阅odom话题获取当前位置
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    
    // 等待首次位置更新
    ros::Rate rate(10); // 10Hz
    while (ros::ok() && !position_updated) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Initial position received!");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "world";
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    for (size_t i = 0; i < goals.size(); ++i) {
        const GoalPoint& target = goals[i];
        
        // 设置目标位置
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = target.x;
        goal.target_pose.pose.position.y = target.y;
        
        ROS_INFO("Sending goal %zu: (%.2f, %.2f)", i+1, target.x, target.y);
        ac.sendGoal(goal);
        
        // 等待机器人接近目标点（1米范围内）
        const double proximity_threshold = 1.0; // 1米
        bool goal_reached = false;
        bool action_completed = false;
        
        ros::Time start_time = ros::Time::now();
        
        while (ros::ok() && !goal_reached && !action_completed) {
            ros::spinOnce(); // 处理回调
            
            // 检查动作状态
            actionlib::SimpleClientGoalState state = ac.getState();
            action_completed = state.isDone();
            
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
            
            // 超时检查（300秒）
            if ((ros::Time::now() - start_time).toSec() > 300.0) {
                ROS_WARN("Timeout for goal %zu, moving to next", i+1);
                break;
            }
            
            // 显示状态
            ROS_INFO_THROTTLE(5, "Goal %zu - Distance: %.2fm, State: %s", 
                             i+1, distance, state.toString().c_str());
            
            rate.sleep();
        }
        
        // 取消当前目标（如果尚未完成）
        if (!goal_reached && !action_completed) {
            ac.cancelGoal();
            ROS_INFO("Cancelled goal %zu", i+1);
        }
        
        // 短暂暂停
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("All goals completed!");
    return 0;
}