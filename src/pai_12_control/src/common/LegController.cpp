#include "../../include/common/LegController.h"
#include <eigen3/Eigen/Core>

// upper level of joint controller
// send data to joint controller

void LegControllerCommand::zero()
{
    tau = Vec5<double>::Zero();
    qDes = Vec5<double>::Zero();
    qdDes = Vec5<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    hiptoeforce = Vec3<double>::Zero();
    kpCartesian = Mat3<double>::Zero();
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat5<double>::Zero();
    kdJoint = Mat5<double>::Zero();
    double kptoe = 0;
    double kdtoe = 0;
}

/*!
 * Zero leg data
 */
void LegControllerData::zero()
{
    q = Vec5<double>::Zero();
    qd = Vec5<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J_force_moment = Mat65<double>::Zero();
    J_force = Mat35<double>::Zero();
    tau = Vec5<double>::Zero();
}

void LegController::zeroCommand()
{
    for (int i = 0; i < 2; i++)
    {
        commands[i].zero();
    }
}

void LegController::updateData(const LowlevelState *state)
{
    for (int leg = 0; leg < 2; leg++)
    {
        for (int j = 0; j < 5; j++)
        {
            switch (j)
            {
            // case 2:
            //     data[leg].q(j) = state->motorState[leg * 5 + j].q+0.25*M_PI;
            //     break;
            // case 3:
            //     data[leg].q(j) = state->motorState[leg * 5 + j].q-0.5*M_PI;
            //     break;
            // case 4:
            //     data[leg].q(j) = state->motorState[leg * 5 + j].q+0.25*M_PI;
            //     break;
            default:
                data[leg].q(j) = state->motorState[leg * 5 + j].q;
                break;
            }
            data[leg].qd(j) = state->motorState[leg * 5 + j].dq;
            data[leg].tau(j) = state->motorState[leg * 5 + j].tauEst;
            std::cout << "motor joint tau " << leg * 5 + j << ": " << data[leg].tau(j) << std::endl;
        }
        computeLegJacobianAndPosition(_biped, data[leg].q,
                                      &(data[leg].J_force_moment),
                                      &(data[leg].J_force), &(data[leg].p), leg);
        std::cout << "1231541" << std::endl;
        data[leg].v = data[leg].J_force * data[leg].qd;
    }
    for (size_t i = 0; i < 3; i++)
    {
          rpy[i] = state->rpy[i];
    }
    
}

void LegController::updateCommand(LowlevelCmd *cmd)
{
    for (int i = 0; i < 2; i++)
    {
        std::cout << "*************" << i << "\n";
        Vec6<double> footForce = commands[i].feedforwardForce;
        for (size_t i = 0; i < 6; i++)
        {
            F_M_arr.data[i] = footForce[i];
        }
        F_M_pub.publish(F_M_arr);
        std::cout << "footForce:\n"
                  << footForce.transpose() << std::endl;
        // 利用雅可比，根据足端力计算关节力矩。这里忽略了腿的动力学，只考虑了静力学
        Vec5<double> legtau = data[i].J_force_moment.transpose() *
                              footForce; // force moment from stance leg
        std::cout << "legtau: \n" << legtau.transpose() << "\n";
        // 通过笛卡尔空间的kp和kd判断需不需要进行摆动腿力矩的计算
        if (commands[i].kpCartesian(0, 0) != 0 ||
            commands[i].kdCartesian(0, 0) != 0)
        {
            // 摆动腿的足端在笛卡尔空间的力（弹簧阻尼模型）
            Vec3<double> footForce_3d =
                commands[i].kpCartesian*(commands[i].pDes - data[i].p) +
                commands[i].kdCartesian * (commands[i].vDes - data[i].v);
            // // print pDes
            // std::cout << "pDes: " << commands[i].pDes.transpose() << std::endl;
            // std::cout << "p: " << data[i].p.transpose() << std::endl;
            // 摆动腿的足端在笛卡尔空间的力转换到关节空间的力矩，此处的J_force和上面的J_force_moment不同
            Vec5<double> swingtau = data[i].J_force.transpose() * footForce_3d;
            // maintain hip angle tracking
            // 单独控制hip-yaw关节，使其保持0角度
            swingtau(0) = kphip0 * (0 - data[i].q(0))-kdhip0*(data[i].qd(0));
            // 单独控制hip-roll关节，补偿机体倾斜的角度，使得脚掌平面平行于地面
            swingtau(1) = kphip1 * (-rpy[0] - data[i].q(1))+kdhip1*(0-data[i].qd(1));
            // std::cout<<"==============roll:\n"<<rpy[0]<<"   "<<data[i].q(1)<<"  "<<swingtau(1) <<std::endl;
            // 单独控制thigh关节，补偿机体前倾的角度
            // swingtau(2) = kphip2 * (-rpy[1] - data[i].q(2))+kdhip2*(0-data[i].qd(2));
            // make sure foot is parallel with the ground
            // 单独控制摆动腿的脚踝关节，使其保持0角度，与地面平行
            swingtau(4) = commands[i].kptoe * (-data[i].q(3) - data[i].q(2) - data[i].q(4));
            for (int j = 0; j < 5; j++)
            {
                legtau(j) += swingtau(j);
            }
        }
        commands[i].tau += legtau;
        // 更新关节力矩
        std::cout << "commands[i].tau: \n" << commands[i].tau.transpose() << "\n";

        for (int j = 0; j < 5; j++)
        {
            cmd->motorCmd[i * 5 + j].tau = commands[i].tau(j);
            cmd->motorCmd[i * 5 + j].q = commands[i].qDes(j);
            cmd->motorCmd[i * 5 + j].dq = commands[i].qdDes(j);
            cmd->motorCmd[i * 5 + j].Kp = commands[i].kpJoint(j, j);
            cmd->motorCmd[i * 5 + j].Kd = commands[i].kdJoint(j, j);
        }
        commands[i].tau << 0, 0, 0, 0, 0; // zero torque command to prevent interference
    }
}

void computeLegJacobianAndPosition(Biped &_biped, Vec5<double> &q, Mat65<double> *J_f_m, Mat35<double> *J_f,
                                   Vec3<double> *p, int leg)
{
    std::cout << "joint num: " << _biped.model_.nq << std::endl;
    // 为什么会有两个雅可比矩阵？
    // J_f_m维度是6*5，J_f维度是3*5，J_f_m标准的雅可比矩阵，J_f是J_f_m的前三列
    // 对于摆动腿，不需要对脚掌的姿态进行控制，所以只需要前三列
    Eigen::VectorXd q_full;
    int foot_l_id = 5;
    int foot_r_id = 10;
    q_full.setZero(10);

    pinocchio::Data::Matrix6x J(6, _biped.model_.nv);
    J.setZero();
    pinocchio::Data::Matrix6x J_leg(6, 5);
    J_leg.setZero();

    if (leg == 0)
    {
        q_full.head(5) = q;
    }
    else
    {
        q_full.tail(5) = q;
    }
    std::cout << "joint num: " << _biped.model_.nq << std::endl;
    pinocchio::forwardKinematics(_biped.model_, _biped.data_, q_full);
    pinocchio::updateFramePlacements(_biped.model_, _biped.data_);
    Vec3<double> pos_foot;
    pos_foot.setZero();
    if (leg == 0)
    {
        pos_foot = _biped.data_.oMi[foot_l_id].translation();
        auto ori_foot = _biped.data_.oMi[foot_l_id].rotation();
        pinocchio::Data::Matrix6x R_leg(6, 6);
        R_leg.setZero();
        R_leg.topLeftCorner(3, 3) = ori_foot;
        R_leg.bottomRightCorner(3, 3) = ori_foot;
        pinocchio::computeJointJacobian(_biped.model_, _biped.data_, q_full,
                                        foot_l_id, J);
        J_leg = J.leftCols(5);
        J_leg = R_leg * J_leg;
    }
    else
    {
        pos_foot = _biped.data_.oMi[foot_r_id].translation();
        auto ori_foot = _biped.data_.oMi[foot_r_id].rotation();
        pinocchio::Data::Matrix6x R_leg(6, 6);
        R_leg.setZero();
        R_leg.topLeftCorner(3, 3) = ori_foot;
        R_leg.bottomRightCorner(3, 3) = ori_foot;
        pinocchio::computeJointJacobian(_biped.model_, _biped.data_, q_full,
                                        foot_r_id, J);
        J_leg = J.rightCols(5);
        J_leg = R_leg * J_leg;
    }
    // 如果指针不为nullptr,则计算雅克比
    if (J_f_m)
    {
        *J_f_m = J_leg;
    }
    if (J_f)
    {
        *J_f = J_leg.topRows(3);
    }
    if (p)
    {
        std::cout << pos_foot << std::endl;
        if (leg == 0)
        {
            // std::cout<<"pinocchio pos:"<<std::endl;
            p->operator()(0) = pos_foot(0) - 0.005;
            p->operator()(1) = pos_foot(1) - _biped.leg_offset_y2;
            p->operator()(2) = pos_foot(2) + 0.02;
            // std::cout<<p->operator()(0)<<" "<<p->operator()(1)<<"
            // "<<p->operator()(2)<<std::endl;
        }
        else
        {
            // std::cout<<"pinocchio pos:"<<std::endl;
            p->operator()(0) = pos_foot(0) - 0.005;
            p->operator()(1) = pos_foot(1) + _biped.leg_offset_y2;
            p->operator()(2) = pos_foot(2) + 0.02;
            // std::cout<<p->operator()(0)<<" "<<p->operator()(1)<<"
            // "<<p->operator()(2)<<std::endl;
        }
    }
}