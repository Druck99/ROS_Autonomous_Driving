#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <cmath>

class ControllerNode
{
public:
    ControllerNode()
      : nh_(""), pnh_("~"),
        current_speed_(0.0), des_vx_(0.0), des_vy_(0.0), des_omega_(0.0),
        received_desired_(false), go_command_(true)  // 初始化为允许行驶
    {
        // 参数
        pnh_.param("wheelbase", wheelbase_, 1.0);
        pnh_.param("max_steer_deg", max_steer_deg_, 20.0);
        pnh_.param("max_acc", max_acc_, 1.89);
        pnh_.param("max_speed", max_speed_, 8.5);
        pnh_.param("Kp_speed", Kp_speed_, 1.0);
        pnh_.param("Kp_brake", Kp_brake_, 1.0);
        pnh_.param("control_rate", rate_hz_, 20.0);

        // 订阅期望速度
        sub_des_ = nh_.subscribe<geometry_msgs::Twist>(
            "/cmd_vel", 1, &ControllerNode::cbDesired, this);
        // 订阅当前速度
        sub_state_ = nh_.subscribe<geometry_msgs::TwistStamped>(
            "/Unity_ROS_message_Rx/OurCar/CoM/twist", 1,
            &ControllerNode::cbState, this);
        // 订阅决策节点的 GO/STOP 信号**
        sub_decision_ = nh_.subscribe<std_msgs::Bool>(
            "/ssc", 1, &ControllerNode::cbDecision, this);

        // 发布控制命令
        pub_ctrl_ = nh_.advertise<simulation::VehicleControl>("car_command", 1);
    }

    void spin()
    {
        ros::Rate rate(rate_hz_);
        while (ros::ok())
        {
            simulation::VehicleControl cmd;

            // —— 优先响应决策节点 —— 
            if (!go_command_)
            {
                // 红灯或紧急停车：立即全刹车
                cmd.Throttle = 0.0f;
                cmd.Brake    = 1.0f;    // 最大制动
                cmd.Steering = 0.0f;
            }
            else if (!received_desired_)
            {
                // 启动时未收到规划器命令，保持初始油门驱动
                cmd.Throttle = 0.5f;
                cmd.Brake    = 0.0f;
                cmd.Steering = 0.0f;
            }
            else
            {
                // —— 原有纵横向控制逻辑 —— 
                // 纵向
                double target_v = std::max(std::min(des_vx_, max_speed_), -max_speed_);
                double err_v    = target_v - current_speed_;
                if (err_v >= 0.0) {
                    cmd.Throttle = std::min(1.0f, float(Kp_speed_ * err_v / max_acc_));
                    cmd.Brake    = 0.0f;
                } else {
                    cmd.Throttle = 0.0f;
                    cmd.Brake    = std::min(1.0f, float(Kp_brake_ * (-err_v) / max_acc_));
                }

                // 横向
                double steer_rad = 0.0;
                if (std::fabs(target_v) > 0.1) {
                    steer_rad = std::atan2(wheelbase_ * des_omega_, target_v);
                }
                double max_rad = max_steer_deg_ * M_PI / 180.0;
                steer_rad = std::max(-max_rad, std::min(max_rad, steer_rad));
                cmd.Steering = float(steer_rad / max_rad);
            }

            cmd.Reserved = 0.0f;
            pub_ctrl_.publish(cmd);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // 规划器期望速度回调
    void cbDesired(const geometry_msgs::Twist::ConstPtr& msg)
    {
        des_vx_    = msg->linear.x;
        des_vy_    = msg->linear.y;
        des_omega_ = msg->angular.z;
        received_desired_ = true;
    }

    // 实际速度回调
    void cbState(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    // **决策节点回调：GO==true 继续行驶，false 刹车**
    void cbDecision(const std_msgs::Bool::ConstPtr& msg)
    {
        go_command_ = msg->data;
        // ROS_INFO_STREAM("Decision command: " << (go_command_ ? "GO" : "STOP"));
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_des_, sub_state_, sub_decision_;
    ros::Publisher  pub_ctrl_;

    double current_speed_, des_vx_, des_vy_, des_omega_;
    bool   received_desired_, go_command_;

    double wheelbase_, max_steer_deg_, max_acc_, max_speed_;
    double Kp_speed_, Kp_brake_, rate_hz_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ControllerNode node;
    node.spin();
    return 0;
}





// #include <ros/ros.h>
// #include <simulation/VehicleControl.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <cmath>

// class ControllerNode
// {
// public:
//     ControllerNode()
//       : nh_(""), pnh_("~"), current_speed_(0.0), des_vx_(0.0), des_vy_(0.0), des_omega_(0.0)
//     {
//         // 1) 从参数服务器读取车辆动力学／控制参数
//         pnh_.param("wheelbase", wheelbase_, 1.0);                  // 轴距[m]
//         pnh_.param("max_steer_deg", max_steer_deg_, 20.0);         // 最大转角[deg]
//         pnh_.param("max_acc", max_acc_, 1.89);                    // 最大加速度[m/s^2]
//         pnh_.param("max_speed", max_speed_, 8.5);                 // 最大速度[m/s]
//         pnh_.param("Kp_speed", Kp_speed_, 1.0);                   // 速度 P 控制
//         pnh_.param("Kp_brake", Kp_brake_, 1.0);                   // 制动 P 控制
//         pnh_.param("control_rate", rate_hz_, 20.0);               // 控制频率[Hz]

//         // 2) 订阅期望速度（规划器输出）
//         sub_des_ = nh_.subscribe<geometry_msgs::Twist>(
//             "/cmd_vel", 1, &ControllerNode::cbDesired, this);     // 更新目标速度

//         // 3) 订阅当前速度（从 INS／里程计）
//         sub_state_ = nh_.subscribe<geometry_msgs::TwistStamped>(
//             "/Unity_ROS_message_Rx/OurCar/CoM/twist", 1,
//             &ControllerNode::cbState, this);                      //更新当前速度

//         // 4) 发布控制命令
//         pub_ctrl_ = nh_.advertise<simulation::VehicleControl>("car_command", 1);
//     }

// void spin()
//     {
//         ros::Rate rate(rate_hz_);
//         while (ros::ok())
//         {
//             simulation::VehicleControl cmd;

//             if (!received_desired_)
//             {
//                 // 启动时未收到规划器命令，保持初始油门驱动
//                 cmd.Throttle = 0.5f;    // 初始油门
//                 cmd.Brake    = 0.0f;
//                 cmd.Steering = 0.0f;
//             }
//             else
//             {
//                 // —— 纵向控制 —— 
//                 // 限制期望速度不超过 max_speed_
//                 double target_v = std::max(std::min(des_vx_, max_speed_), -max_speed_);
//                 double err_v = target_v - current_speed_;
//                 // 简单 P 控制，正向用油门，反向用刹车
//                 if (err_v >= 0.0) {
//                     cmd.Throttle = std::min(1.0f, float(Kp_speed_ * err_v / max_acc_));
//                     cmd.Brake    = 0.0f;
//                 } else {
//                     cmd.Throttle = 0.0f;
//                     cmd.Brake    = std::min(1.0f, float(Kp_brake_ * (-err_v) / max_acc_));
//                 }

//                 // —— 横向（转向）控制 —— 
//                 // 使用 Ackermann 模型：delta = atan(L * ω / v)
//                 double steer_rad = 0.0;
//                 if (std::fabs(target_v) > 0.1) {
//                     steer_rad = std::atan2(wheelbase_ * des_omega_, target_v);
//                 }
//                 // 限制角度，并归一化到[-1,1]
//                 double max_rad = max_steer_deg_ * M_PI / 180.0;
//                 steer_rad = std::max(-max_rad, std::min(max_rad, steer_rad));
//                 cmd.Steering = float(steer_rad / max_rad);
//             }

//             cmd.Reserved = 0.0f;

//             // 发布
//             pub_ctrl_.publish(cmd);

//             ros::spinOnce();            //检查有没有新消息触发回调
//             rate.sleep();               //等待下一次循环
//         }
//     }

// private:
//     // 回调：规划器期望速度
//     void cbDesired(const geometry_msgs::Twist::ConstPtr& msg)
//     {
//         des_vx_    = msg->linear.x;
//         des_vy_    = msg->linear.y;  // 暂不使用侧向速度
//         des_omega_ = msg->angular.z;
//         received_desired_ = true;    // 收到过一次期望命令
//     }
//     // 回调：车辆实际速度
//     void cbState(const geometry_msgs::TwistStamped::ConstPtr& msg)
//     {
//         current_speed_ = msg->twist.linear.x;
//     }

//     ros::NodeHandle nh_, pnh_;
//     ros::Subscriber sub_des_, sub_state_;
//     ros::Publisher  pub_ctrl_;

//     double current_speed_;
//     double des_vx_, des_vy_, des_omega_;
//     bool   received_desired_;      // 是否收到过规划器命令

//     // 参数
//     double wheelbase_, max_steer_deg_, max_acc_, max_speed_;
//     double Kp_speed_, Kp_brake_, rate_hz_;
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "controller_node");
//     ControllerNode node;
//     node.spin();
//     return 0;
// }


// constexpr float loop_interval = 0.05;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "dummy_controller_node");
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<simulation::VehicleControl>("car_command", 1);
//     ros::Rate loop_rate(1 / loop_interval);
//     float elapsed_time = 0.0f;

//     while (ros::ok())
//     {
//         simulation::VehicleControl msg;
//         msg.Throttle = 0.5f; // Throttle value from -1 to 1, this is the torque applied to the motors
//         msg.Steering =  sin(6.28 * elapsed_time) * 0.5; //Steering value from -1 to 1, in which: positive value <=> turning right
//         msg.Brake = 0.0f; // Brake value from 0 to 1, this will apply brake torque to stop the car
//         msg.Reserved = 0.0f; // Not used!

//         pub.publish(msg);

//         ros::spinOnce();
//         loop_rate.sleep();
//         elapsed_time += loop_interval;

//     }

//     return 0;
// }

