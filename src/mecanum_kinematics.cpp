#include "hero_chassis_controller/mecanum_kinematics.h"

namespace hero_chassis_controller
{

    Mecanum::Mecanum(double r, double len_x, double len_y)
    {
        this->r = r;
        this->len_x = len_x;
        this->len_y = len_y;
    }

    // 小车想往前走 vx、往左走 vy、转 wz，四个轮子该转多快
    void Mecanum::inverse(double vx, double vy, double wz, double wheel[4])
    {
        double k = len_x / 2.0 + len_y / 2.0;

        wheel[0] = (vx - vy - k * wz) / r;
        wheel[1] = (vx + vy + k * wz) / r;
        wheel[2] = (vx + vy - k * wz) / r;
        wheel[3] = (vx - vy + k * wz) / r;
    }

    // 四个轮子实际转多快，反推出小车现在走多快
    void Mecanum::forward(double wheel[4], double& vx, double& vy, double& wz)
    {
        double k = len_x / 2.0 + len_y / 2.0;

        vx = r * (wheel[0] + wheel[1] + wheel[2] + wheel[3]) / 4.0;
        vy = r * (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) / 4.0;
        wz = r * (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) / (4.0 * k);
    }

}  // namespace hero_chassis_controller