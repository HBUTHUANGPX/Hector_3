#ifndef _CALCULATE_JOCOBIA_H_
#define _CALCULATE_JOCOBIA_H_
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include "../common/cppTypes.h"
#include "../common/LegController.h"
class calculate_jocobia
{
private:
    Eigen::VectorXd q_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd q_full;
    pinocchio::Data::Matrix6x J, J_f_m;
    pinocchio::Data::Matrix3x J_f;
    pinocchio::Data::Matrix6x M_leg;
    Vec3<double> pos_foot;
    Vec5<double> joint_torque;
    ros::NodeHandle nh;
    ros::Publisher joint_pub;
    sensor_msgs::JointState joint_state;
    // tf2_ros::TransformBroadcaster br;
    // geometry_msgs::TransformStamped trS;
public:
    calculate_jocobia();
    ~calculate_jocobia();
    void calculat_jocobia(int leg_index);
    void fresh_joint(int leg_index, Vec5<double> &q);
    void calculat_leg_J(int leg_index);
    void calculate_endpoint_vel_and_ang_vel(int leg_index,Vec5<double> &dq);
    void calculate_joint_torque_3(int leg_index,Vec6<double> &F_M);
    void publish_joint_state();
    void print_jointPlacements();
    void print_joint_id_with_name();
    void my_calculate_jocobia();
    void print_link_id_with_name();
};
#endif