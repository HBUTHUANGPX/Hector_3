#include "../../include/jocobia/calculate_jocobia.h"

calculate_jocobia::calculate_jocobia()
{
    std::string path = ros::package::getPath("pai_12dof_v2_0312");
    path += "/urdf/pai_sit.urdf";
    pinocchio::urdf::buildModel(path, model_);
    data_ = pinocchio::Data(model_);
    q_full.setZero(10);
    J = pinocchio::Data::Matrix6x(6, model_.nv);
    J.setZero();
    J_f_m = pinocchio::Data::Matrix6x(6, 5);
    J_f_m.setZero();
    J_f = pinocchio::Data::Matrix3x(3, 5);
    J_f.setZero();
    M_leg = pinocchio::Data::Matrix6x(6, 6);
    M_leg.setZero();
    pos_foot.setZero();
    joint_torque.setZero();
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"l_hip_yaw_joint", "l_hip_roll_joint", "l_thigh_joint", "l_calf_joint", "l_ankle_pitch_joint",
                        "r_hip_yaw_joint", "r_hip_roll_joint", "r_thigh_joint", "r_calf_joint", "r_ankle_pitch_joint"}; // 替换为你的关节名称
    joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0}; // 替换为实际的关节位置
}

calculate_jocobia::~calculate_jocobia()
{
}
void calculate_jocobia::fresh_joint(int leg_index, Vec5<double> &q)
{
    if (leg_index == 0)
    {
        q_full.head(5) = q;
        for (size_t i = 0; i < 5; i++)
        {
            joint_state.position[i] = q[i];
            joint_state.header.stamp = ros::Time::now();
            publish_joint_state();
        }
    }
    else
    {
        q_full.tail(5) = q;
        for (size_t i = 0; i < 5; i++)
        {
            joint_state.position[i + 5] = q[i];
            joint_state.header.stamp = ros::Time::now();
            publish_joint_state();
        }
    }
    pinocchio::forwardKinematics(model_, data_, q_full);
    pinocchio::updateFramePlacements(model_, data_);
}
void calculate_jocobia::calculat_jocobia(int leg_index)
{
    if (leg_index == 0)
    {
        // pinocchio::computeJointJacobian(model_, data_, q_full, 5, J);
        pinocchio::computeFrameJacobian(model_, data_, q_full, 12, J);
        // std::cout << "jocobia:\n"
        //           << J.leftCols(5) << std::endl;
    }
    else
    {
        // pinocchio::computeJointJacobian(model_, data_, q_full, 10, J);
        pinocchio::computeFrameJacobian(model_, data_, q_full, 22, J);
        // std::cout << "jocobia:\n"
        //           << J.rightCols(5) << std::endl;
    }
}
void calculate_jocobia::calculat_leg_J(int leg_index)
{
    calculat_jocobia(leg_index);
    pos_foot = data_.oMi[5 + 5 * leg_index].translation();
    std::cout << "index:\n"
              << 5 + 5 * leg_index << std::endl;
    std::cout << "pos_foot:\n"
              << pos_foot << std::endl;
    M_leg.topLeftCorner(3, 3) = data_.oMi[5 + 5 * leg_index].rotation();
    M_leg.bottomRightCorner(3, 3) = data_.oMi[5 + 5 * leg_index].rotation();
    if (leg_index == 0)
    {
        J_f_m = M_leg * J.leftCols(5);
    }
    else
    {
        J_f_m = M_leg * J.rightCols(5);
    }
    J_f = J_f_m.topRows(3);
}
void calculate_jocobia::calculate_endpoint_vel_and_ang_vel(int leg_index, Vec5<double> &dq)
{
    if (leg_index == 0)
    {
        // std::cout << "left toe ang vel and vel:\n"
        //           << J_f_m * dq<< std::endl;
    }
    else
    {
        // std::cout << "right toe ang vel and vel:\n"
        //           << J_f_m * dq<< std::endl;
    }
}
void calculate_jocobia::calculate_joint_torque_3(int leg_index, Vec6<double> &F_M)
{
    // std::cout << "J_f_m.transpose():\n"
    //           << J_f_m.transpose() << std::endl;
    // std::cout << "F_M:\n"
    //           << F_M << std::endl;
    joint_torque = (J_f_m.transpose() * F_M);
    for (size_t i = 0; i < 5; i++)
    {
        printf("%8.6lf  ", joint_torque(i));
    }
    std::cout << std::endl;
}
void calculate_jocobia::publish_joint_state()
{
    joint_pub.publish(joint_state);
}
void calculate_jocobia::print_jointPlacements()
{
    for (size_t joint_id = 1; joint_id < model_.njoints; ++joint_id)
    {
        // 获取关节的偏移向量
        const Eigen::Vector3d &translation = model_.jointPlacements[joint_id].translation();
        // 打印关节的偏移向量
        std::cout << "Joint ID: " << joint_id << " Offset: " << translation.transpose() << " norm:" << translation.norm() << std::endl;
    }
}
void calculate_jocobia::print_joint_id_with_name()
{
    for (size_t joint_id = 0; joint_id < model_.njoints; joint_id++)
    {
        std::string joint_name = model_.names[joint_id];
        std::cout << "Joint ID: " << joint_id << " has the name: " << joint_name << std::endl;
    }
    for (size_t joint_id = 0; joint_id < model_.njoints; joint_id++)
    {
        std::string joint_name = model_.names[joint_id];
        std::cout << "\"" << joint_name << "\""
                  << " ,";
    }
    std::cout << std::endl;
}
void calculate_jocobia::print_link_id_with_name()
{
    for (size_t i = 0; i < model_.frames.size(); ++i)
    {
        const pinocchio::Frame &frame = model_.frames[i];
        if (frame.type == pinocchio::BODY)
        {
            std::cout << "Link ID:" << i << " Link name: " << frame.name << std::endl;
        }
    }
}

void calculate_jocobia::my_calculate_jocobia()
{
    Eigen::MatrixXd J(6, 5); // 6行（线性+角速度），nv列（关节数量）
    J.setZero();
    for (size_t joint_id = 1; joint_id <= 5; ++joint_id)
    {
        // 获取关节的位置和方向
        pinocchio::SE3 joint_placement = data_.oMi[joint_id];
        // std::cout<<"joint_placement:\n"<<joint_placement<<std::endl;
        // 计算从关节到末端执行器的向量
        Eigen::Vector3d joint_to_effector = data_.oMf[5].translation() - joint_placement.translation();
        // std::cout<<"data_.oMf[5]:\n"<<data_.oMf[5]<<std::endl;
        Eigen::Vector3d joint_axis;
        if (model_.joints[joint_id].shortname() == "JointModelRX")
        {
            joint_axis = joint_placement.rotation().col(0); // 绕x轴旋转
        }
        else if (model_.joints[joint_id].shortname() == "JointModelRY")
        {
            joint_axis = joint_placement.rotation().col(1); // 绕y轴旋转
        }
        else if (model_.joints[joint_id].shortname() == "JointModelRZ")
        {
            joint_axis = joint_placement.rotation().col(2); // 绕z轴旋转
        }
        J.block<3, 1>(0, joint_id - 1) = joint_axis.cross(joint_to_effector); // 线性速度部分
        J.block<3, 1>(3, joint_id - 1) = joint_axis;                          // 角速度部分
    }

    // 打印雅可比矩阵
    std::cout << "Jacobian matrix:" << std::endl
              << J << std::endl;
}