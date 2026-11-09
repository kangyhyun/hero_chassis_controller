#include "hero_chassis_controller/hero_chassis_controller.h"
//引入头文件，告诉编译器HeroChassisController在哪里，init和update的参数和返回值

#include <pluginlib/class_list_macros.hpp>
//将c++类导出成pluginlib可加载的插件
/*namespace*/
namespace hero_chassis_controller {

    bool HeroChassisController::init(
        hardware_interface::EffortJointInterface*,
        ros::NodeHandle&,
        ros::NodeHandle&) {
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
        ROS_INFO_STREAM_ONCE(
            "HeroChassisController update() is running.");
    }

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(
    hero_chassis_controller::HeroChassisController,//真正实现的控制器类
    controller_interface::ControllerBase);//它的ROS插件基类


/* namespace */
