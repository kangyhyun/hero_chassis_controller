#pragma once

#include <string>

#include "controller_interface/controller.h"
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>


namespace hero_chassis_controller {

class HeroChassisController
    :public controller_interface::Controller< //控制器基础类别
        hardware_interface::EffortJointInterface> //控制关节的接口类别
{
public:
    bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
              ros::NodeHandle& root_nh,
              ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time,
              const ros::Duration& period) override;
private:
    hardware_interface::JointHandle left_front_joint_;
    hardware_interface::JointHandle right_front_joint_;
    hardware_interface::JointHandle left_back_joint_;
    hardware_interface::JointHandle right_back_joint_;
    control_toolbox::Pid left_front_pid_;
    control_toolbox::Pid right_front_pid_;
    control_toolbox::Pid left_back_pid_;
    control_toolbox::Pid right_back_pid_;
};
} /* namespace hero_chassis_controller*/