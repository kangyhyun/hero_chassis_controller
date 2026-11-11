#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
namespace hero_chassis_controller
{

// 读一个轮子的 PID 参数
static void readPid(ros::NodeHandle nh, std::string name,
                    control_toolbox::Pid& pid)
{
    double p = nh.param(name + "/p", 1.0);
    double i = nh.param(name + "/i", 0.0);
    double d = nh.param(name + "/d", 0.0);
    double limit = nh.param(name + "/i_clamp", 1.0);
    pid.initPid(p, i, d, limit, -limit);
}

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort,
    ros::NodeHandle& root_nh,
    ros::NodeHandle& nh)
{

    // 1. 拿四个轮子的句柄
    joint_lf = effort->getHandle("left_front_wheel_joint");
    joint_rf = effort->getHandle("right_front_wheel_joint");
    joint_lb = effort->getHandle("left_back_wheel_joint");
    joint_rb = effort->getHandle("right_back_wheel_joint");

    // 2. 读参数
    wheel_r = nh.param("wheel_radius", 0.07625);
    wheel_x = nh.param("wheel_base", 0.4);
    wheel_y = nh.param("wheel_track", 0.4);
    use_global = nh.param("world_frame_mode", false);
    max_wheel_speed = nh.param("max_wheel_speed", 50.0);//限制最大轮子速度
    // 3. 套公式
    mecanum = Mecanum(wheel_r, wheel_x, wheel_y);

    // 4. 四个 PID 读参数
    readPid(nh, "left_front_pid", pid_lf);
    readPid(nh, "right_front_pid", pid_rf);
    readPid(nh, "left_back_pid", pid_lb);
    readPid(nh, "right_back_pid", pid_rb);

    // 5. 订阅速度指令，发布里程计
    odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 10);
    cmd_sub = root_nh.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 1, &HeroChassisController::cmdCallback, this);

    ROS_INFO("HeroChassisController started");
    return true;
}

// 收到 /cmd_vel 就把消息存起来
void HeroChassisController::cmdCallback(
    const geometry_msgs::Twist::ConstPtr& msg)
{

    geometry_msgs::Twist t = *msg;

    //世界系模式，先把速度转到车身方向
    if (use_global)
    {
        geometry_msgs::Vector3Stamped in;
        geometry_msgs::Vector3Stamped out;
        in.header.frame_id = odom_frame;
        in.header.stamp = ros::Time(0);
        in.vector = msg->linear;

        try
        {
            tf_listener.transformVector(base_frame, in, out);
            t.linear = out.vector;
        } catch (tf::TransformException& e)
        {
            ROS_WARN("tf error");
            return;
        }
    }

    cmd = t;
}

// 每个控制周期跑一次
void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& dt)
{
    double d = dt.toSec();

    // 1. 逆运动学：车想怎么走 -> 四个轮子转多快
    double speed[4];
    mecanum.inverse(cmd.linear.x, cmd.linear.y, cmd.angular.z, speed);
    //限速
    speed[0] = std::max(-max_wheel_speed, std::min(max_wheel_speed, speed[0]));
    speed[1] = std::max(-max_wheel_speed, std::min(max_wheel_speed, speed[1]));
    speed[2] = std::max(-max_wheel_speed, std::min(max_wheel_speed, speed[2]));
    speed[3] = std::max(-max_wheel_speed, std::min(max_wheel_speed, speed[3]));
    // 2. 四个轮子各算一次力矩，写法一样
    double e;
    double u;

    // 左前
    e = speed[0] - joint_lf.getVelocity();
    u = pid_lf.computeCommand(e, dt);
    joint_lf.setCommand(u);

    // 右前
    e = speed[1] - joint_rf.getVelocity();
    u = pid_rf.computeCommand(e, dt);
    joint_rf.setCommand(u);

    // 左后
    e = speed[2] - joint_lb.getVelocity();
    u = pid_lb.computeCommand(e, dt);
    joint_lb.setCommand(u);

    // 右后
    e = speed[3] - joint_rb.getVelocity();
    u = pid_rb.computeCommand(e, dt);
    joint_rb.setCommand(u);

    // 3. 正运动学：四个轮子实际转多快 -> 车实际走多快
    double real[4];
    real[0] = joint_lf.getVelocity();
    real[1] = joint_rf.getVelocity();
    real[2] = joint_lb.getVelocity();
    real[3] = joint_rb.getVelocity();

    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    mecanum.forward(real, vx, vy, wz);

    // 4. 把速度按当前朝向转到 odom 系，再累加位置
    pos_x = pos_x + (vx * std::cos(pos_yaw) - vy * std::sin(pos_yaw)) * d;
    pos_y = pos_y + (vx * std::sin(pos_yaw) + vy * std::cos(pos_yaw)) * d;
    pos_yaw = pos_yaw + wz * d;

    // 5. 发出去
    sendOdom(time, vx, vy, wz);
}

// 发布 /odom到base_link
void HeroChassisController::sendOdom(const ros::Time& time,
                                     double vx, double vy, double wz)
{
    // 先发坐标变换
    tf::Transform trans;
    trans.setOrigin(tf::Vector3(pos_x, pos_y, 0.0));
    trans.setRotation(tf::createQuaternionFromYaw(pos_yaw));
    tf_broadcaster.sendTransform(
        tf::StampedTransform(trans, time, odom_frame, base_frame));

    // 再填里程计消息
    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_frame;
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos_yaw);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;
    odom_pub.publish(odom);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)