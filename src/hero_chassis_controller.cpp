#include "hero_chassis_controller/hero_chassis_controller.h"
//引入头文件，告诉编译器HeroChassisController在哪里，init和update的参数和返回值

#include <pluginlib/class_list_macros.hpp>
//将c++类导出成pluginlib可加载的插件
/*namespace*/
namespace hero_chassis_controller {

    bool HeroChassisController::init(
        hardware_interface::EffortJointInterface* effort_joint_interface,
        ros::NodeHandle&,
        ros::NodeHandle&)
    {
        left_front_joint_=
            effort_joint_interface->getHandle("left_front_wheel_joint");

        right_front_joint_=
            effort_joint_interface->getHandle("right_front_wheel_joint");

        left_back_joint_=
            effort_joint_interface->getHandle("left_back_wheel_joint");

        right_back_joint_ =
            effort_joint_interface->getHandle("right_back_wheel_joint");

        ROS_INFO_STREAM("HeroChassisController initialized.");//打印初始化信息
        return true;
    }
    /*它在控制器被加载时调用一次。三个参数分别是：
 EffortJointInterface*：用来获取四个轮子的 JointHandle；
 root_nh：ROS 根命名空间；
 controller_nh：控制器自己的命名空间，用来读取 PID 等参数*/


    void HeroChassisController::update(
        const ros::Time&,
        const ros::Duration&) {
        left_front_joint_.setCommand(0.0);
        right_front_joint_.setCommand(0.0);
        left_back_joint_.setCommand(0.0);
        right_back_joint_.setCommand(0.0);//让四个轮子输出 0 力矩，保持静止

        ROS_INFO_STREAM_ONCE(
                "HeroChassisController update() is running.");
        }

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(
    hero_chassis_controller::HeroChassisController,//真正实现的控制器类
    controller_interface::ControllerBase);//它的ROS插件基类


/* namespace */
