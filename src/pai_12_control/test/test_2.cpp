#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <iostream>
#include <ros/package.h>
#include "../include/common/cppTypes.h"
#include "../include/common/LegController.h"

#include "../include/jocobia/calculate_jocobia.h"
struct LegData{
        LegData() {}
        Vec5<double> q, qd,tau;//q是关节角度 qd是关节速度
        Vec3<double> p, v; //p是位置，v是速度
    };
int main()
{
    calculate_jocobia a;
    LegData data[2];
    for (int leg = 0; leg < 2; leg++)
    {
        for (int j = 0; j < 5; j++)
        {
            data[leg].q(j) = 0.0;
            data[leg].qd(j) = 0.1;
            data[leg].tau(j) = 0.1;
        }
        a.fresh_joint(leg, data[leg].q);
        a.calculat_leg_J(leg);
    }
    return 0;
}