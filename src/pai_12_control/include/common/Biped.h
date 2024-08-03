#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

#include <vector>
#include "cppTypes.h"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <vector>
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>

class Biped
{
public:
    void setBiped()
    {

        mass = 4.6;
        leg_offset_x2 = 0.0;
        ros::param::get("Biped/hip_offset", leg_offset_y2);
        leg_offset_z2 = 0.0;
        std::string path = ros::package::getPath("pai_12dof_v2_0312");
        path += "/urdf/pai_sit.urdf";
        // path += "/xacro/pai_s.urdf";
        pinocchio::urdf::buildModel(path, model_);
        data_ = pinocchio::Data(model_);
        q_.setZero(10);
    }
    int robot_index; // 1 for Aliengo, 2 for A1
    double leg_offset_x2;
    double leg_offset_y2;
    double leg_offset_z2;
    double mass;
    Eigen::VectorXd q_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    Vec3<double> getHip2Location(int leg)
    {
        assert(leg >= 0 && leg < 2);
        Vec3<double> pHip2 = Vec3<double>::Zero();
        if (leg == 0)
        {
            pHip2(0) = leg_offset_x2;
            pHip2(1) = leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        if (leg == 1)
        {
            pHip2(0) = leg_offset_x2;
            pHip2(1) = leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        return pHip2;
    };
};

#endif
