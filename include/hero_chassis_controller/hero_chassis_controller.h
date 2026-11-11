#pragma once
#include "controller_interface/controller.h"
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include "hero_chassis_controller/mecanum_kinematics.h"
#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace hero_chassis_controller
{

    class HeroChassisController
        : public controller_interface::Controller<
              hardware_interface::EffortJointInterface>
    {
    public:
        // 控制器加载时调用一次
        bool init(hardware_interface::EffortJointInterface* effort,
                  ros::NodeHandle& root_nh,
                  ros::NodeHandle& nh) override;

        // 每个控制周期调用一次
        void update(const ros::Time& time,
                    const ros::Duration& dt) override;

    private:
        // 收到 /cmd_vel 时调用
        void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

        // 发布里程计
        void sendOdom(const ros::Time& time, double vx, double vy, double wz);

        // 四个轮子的句柄
        hardware_interface::JointHandle joint_lf;
        hardware_interface::JointHandle joint_rf;
        hardware_interface::JointHandle joint_lb;
        hardware_interface::JointHandle joint_rb;

        // 四个轮子的 PID
        control_toolbox::Pid pid_lf;
        control_toolbox::Pid pid_rf;
        control_toolbox::Pid pid_lb;
        control_toolbox::Pid pid_rb;

        // 算麦轮公式的对象
        Mecanum mecanum;

        ros::Subscriber cmd_sub;
        ros::Publisher odom_pub;
        tf::TransformListener tf_listener;
        tf::TransformBroadcaster tf_broadcaster;

        // 最近一次收到的速度指令
        geometry_msgs::Twist cmd;

        // 车身尺寸，来自 URDF
        double wheel_r = 0.07625;
        double wheel_x = 0.4;
        double wheel_y = 0.4;
        double max_wheel_speed=50.0;


        // 速度模式：true 表示世界系，false 表示车身系
        bool use_global = false;
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";

        // 里程计算出来的位置和朝向
        double pos_x = 0.0;
        double pos_y = 0.0;
        double pos_yaw = 0.0;
    };

}  // namespace hero_chassis_controller