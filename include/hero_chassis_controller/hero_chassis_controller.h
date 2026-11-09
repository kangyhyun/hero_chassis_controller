#pragma once

#include <string>

#include "controller_interface/controller.h"
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>


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
};
} /* namespace hero_chassis_controller*/