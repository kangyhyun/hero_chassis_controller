# hero_chassis_controller

## 简介
A simple four-wheel mecanum chassis controller for ROS Noetic<br >

简略的四轮麦克纳姆轮底盘 ros_control 控制器，运行在 ROS Noetic + Gazebo 仿真环境中。

**关键词**：ROS, ros_control

### License

基于
[BSD 3-Clause license](LICENSE).编译<br/>
**作者：kangyhyun<br/>
维护者：kangyhyun<br/>
邮箱：kang.080016@qq.com**

## 安装

### 功能

- 四轮独立 PID 速度闭环，使用 `control_toolbox::Pid`
- PID 参数可通过 YAML 配置
- 逆运动学：订阅 `/cmd_vel`，计算四个麦轮的目标转速
- 正运动学 + 里程计：根据轮子实际转速积分出底盘位姿，发布 `/odom`
- 发布 `odom -> base_link` 坐标变换
- 支持“车身坐标系”和“世界坐标系”两种速度模式
- 限制轮子最大转速，防止指令过大

### 环境

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

### 依赖
 - [rm_template](https://github.com/gdut-dynamic-x/rm_template),
- [simple_chassis_controller](https://github.com/Bluefairy-ljy/simple_chassis_controller)
- roscpp
- controller_interface
- hardware_interface
- pluginlib
- control_toolbox
- geometry_msgs
- nav_msgs
- tf

## 编译
    cd catkin_workspace/src
    git clone git@github.com:kangyhyun/hero_chassis_controller.git
    # git clone https://github.com/kangyhyun/hero_chassis_controller.git
    cd ../
    rosdep install --from-paths . --ignore-src
    catkin build
    source devel/setup.bash

## Config 文件

Config file config
* **controllers.yaml** hero_chassis_controller 和 joint_state_controller.的参数

## Launch 文件

* 
    **hero_chassis_controller.launch:**仅英雄底盘仿真与简易底盘控制器
    

## 测试

建议新开终端测试，检测环境是否加载
    
    source /opt/ros/noetic/setup.bash
    source ~/chassis_ws/devel/setup.bash

### 运行

    roslaunch hero_chassis_controller hero_chassis_controller.launch

注意：launch 文件里设置了 paused="true"，Gazebo 打开后是暂停状态，
必须点一下左下角的play按钮，仿真时间才会开始走，控制器才会执行 update()并开始移动。

简单测试指令
包含：前进，左移，逆时针移动，停止

        # 前进
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}'
    # 左移
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {y: 0.2}}'
    # 原地逆时针旋转
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{angular: {z: 0.5}}'
    # 停止
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{}'


### 查看

    #查看里程计
    rostopic echo /odom
    rosrun tf tf_echo odom base_link
    
    #查看四个轮子的实时转速
    rostopic echo /joint_states

## Bugs & 功能请求

请在本仓库的 [Issue Tracker](https://github.com/kangyhyun/hero_chassis_controller/issues) 提交问题和需求。