#include <ros/ros.h>
#include <nav_msgs/Odometry.h>                       // Odometry 消息定义:contentReference[oaicite:8]{index=8}
#include <geometry_msgs/PoseStamped.h>               // PoseStamped 消息:contentReference[oaicite:9]{index=9}
#include <geometry_msgs/TwistStamped.h>              // TwistStamped 消息:contentReference[oaicite:10]{index=10}
#include <geometry_msgs/PoseWithCovariance.h>        // PoseWithCovariance:contentReference[oaicite:11]{index=11}
#include <geometry_msgs/TwistWithCovariance.h>       // TwistWithCovariance:contentReference[oaicite:12]{index=12}

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // tf2::doTransform
#include <tf2/exceptions.h>

class OdometryPublisher
{
public:
  OdometryPublisher()
    : tf_buffer_(), tf_listener_(tf_buffer_)  // 在构造函数初始化
  {
    // 订阅 pose 和 twist 主题
    pose_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10,
                              &OdometryPublisher::poseCallback, this);
    twist_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 10,
                               &OdometryPublisher::twistCallback, this);

    // 发布 odom 主题
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

    // 定时器，每 0.02 s 发布一次 Odometry（可根据需求调整）  
    timer_ = nh_.createTimer(ros::Duration(0.02),
                             &OdometryPublisher::timerCallback, this);
  }

private:
  tf2_ros::Buffer        tf_buffer_;         // TF2 缓冲区
  tf2_ros::TransformListener tf_listener_;   // 监听器，自动填充缓冲区

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_, twist_sub_;
  ros::Publisher odom_pub_;
  ros::Timer timer_;

  geometry_msgs::PoseStamped latest_pose_;
  geometry_msgs::TwistStamped latest_twist_;
  bool pose_received_ = false, twist_received_ = false;

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    latest_pose_ = *msg;
    pose_received_ = true;
  }

  void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    latest_twist_ = *msg;
    twist_received_ = true;
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (!pose_received_ || !twist_received_) return;
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = latest_pose_.header.frame_id;
    // odom.header.frame_id = "world";
    odom.child_frame_id = "OurCar/INS";  // 可根据实际坐标系修改
    // odom.child_frame_id = "base_link";

    // 填充位姿
    odom.pose.pose = latest_pose_.pose;
    // 默认协方差设为零，若有传感器协方差可在此处赋值
    for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
    
    // 在发布速度前，先获取从传入速度消息所属坐标系（latest_twist_.header.frame_id）
    // 到本体坐标系（如 "base_link"）的变换
    geometry_msgs::TransformStamped tf_stamped;
    try {
      // 使用 lookupTransform 可以获取所需的坐标系关系，并在缓冲区中等待直到变换可用
      tf_stamped = tf_buffer_.lookupTransform(
        "OurCar/INS",
        // "base_link",                              // 目标坐标系
        latest_twist_.header.frame_id,            // 源坐标系
        ros::Time(0),                             // 最新可用变换
        ros::Duration(0.1));                      // 最多等待 0.1s
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Transform failed: %s", ex.what());
      return;
    }

    // 准备线速度 Vector3Stamped
    geometry_msgs::Vector3Stamped lin_in, lin_out;
    lin_in.header    = latest_twist_.header;
    lin_in.vector    = latest_twist_.twist.linear;
    tf2::doTransform(lin_in, lin_out, tf_stamped);

    // 准备角速度 Vector3Stamped
    geometry_msgs::Vector3Stamped ang_in, ang_out;
    ang_in.header    = latest_twist_.header;
    ang_in.vector    = latest_twist_.twist.angular;
    tf2::doTransform(ang_in, ang_out, tf_stamped);

    // 重组 TwistStamped
    geometry_msgs::TwistStamped twist_out;
    twist_out.header = lin_out.header;  // 或 ang_out.header
    twist_out.twist.linear  = lin_out.vector;
    twist_out.twist.angular = ang_out.vector;

    // 填充到 Odometry 并发布
    odom.twist.twist = twist_out.twist;

    // geometry_msgs::TwistStamped twist_in  = latest_twist_;
    // geometry_msgs::TwistStamped twist_out;
    // tf2::doTransform(twist_in, twist_out, tf_stamped);


    // try {
    //   // 直接用已有的 TransformStamped 去做变换
    //   tf_buffer_.transform(
    //     twist_in,     // 输入
    //     twist_out,    // 输出
    //     "base_link"
    //     // tf_stamped,   // 直接传入 TransformStamped
    //     // ros::Duration(0.0)  // 这里的 timeout 通常设为 0，表示不再等待
    //   );  // :contentReference[oaicite:2]{index=2}
    // }
    // catch (tf2::TransformException &ex) {
    //   ROS_WARN("Transform with stamped failed: %s", ex.what());
    // }

    // 填充速度
    odom.twist.twist = twist_out.twist;
    for (int i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;

    odom_pub_.publish(odom);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  OdometryPublisher node;
  ros::spin();
  return 0;
}
