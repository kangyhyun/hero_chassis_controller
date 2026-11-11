//
// Created by kang on 2026/11/09.
//麦克纳姆轮头文件
#ifndef HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H
#define HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H
namespace hero_chassis_controller
{

    // 四轮麦轮底盘运动学。轮序固定为：左前、右前、左后、右后。
    // 坐标系遵循 ROS 约定：x 向前，y 向左，z 向上，wz 逆时针为正。
    class Mecanum

    {
    public:
        Mecanum(double wheel_radius = 0.07625, double wheel_base = 0.4,
                          double wheel_track = 0.4);

        // 底盘速度 -> 四个轮子的目标角速度
        void inverse(double vx, double vy, double wz, double wheel_speed[4]) ;

        // 四个轮子的实际角速度 -> 底盘速度
        void forward(double wheel_speed[4], double& vx, double& vy,double& wz) ;

    private:
        double r;//半径
        double len_x;//前后距离
        double len_y;//左右间距
    };

}  // class Mecanum
//namespace hero_chassis_controller


#endif //HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H
