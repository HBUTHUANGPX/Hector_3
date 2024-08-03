#include <iostream>
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"

using namespace ori;
using Eigen::Dynamic;
#include <ros/ros.h>
/* =========================== Controller ============================= */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) : iterationsBetweenMPC(_iterations_between_mpc),
                                                                                    horizonLength(10),
                                                                                    dt(_dt),
                                                                                    walking(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Walking"),
                                                                                    standing(horizonLength, Vec2<int>(0, 0), Vec2<int>(10, 10), "Standing")
{
  hight = 0.35;
  gaitNumber = 1;
  dtMPC = dt * iterationsBetweenMPC;
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;
  ros::param::get("ConvexMPCLocomotion/run/Kp", Kp_param);
  std::cout<<"ConvexMPCLocomotion/run/Kp:"<<Kp_param<<std::endl;
  ros::param::get("ConvexMPCLocomotion/run/Kd", Kd_param);
  std::cout<<"ConvexMPCLocomotion/run/Kd:"<<Kd_param<<std::endl;
  ros::param::get("ConvexMPCLocomotion/run/kptoe", Kptoe_param);
  std::cout<<"ConvexMPCLocomotion/run/kptoe:"<<Kptoe_param<<std::endl;
  ros::param::get("ConvexMPCLocomotion/run/kdtoe", Kdtoe_param);
  std::cout<<"ConvexMPCLocomotion/run/kdtoe:"<<Kdtoe_param<<std::endl;
  ros::param::get("ConvexMPCLocomotion/run/Q_roll", Q_roll);
  ros::param::get("ConvexMPCLocomotion/run/Q_pitch", Q_pitch);
  ros::param::get("ConvexMPCLocomotion/run/Q_yaw", Q_yaw);
  ros::param::get("ConvexMPCLocomotion/run/Q_x", Q_x);
  ros::param::get("ConvexMPCLocomotion/run/Q_y", Q_y);
  ros::param::get("ConvexMPCLocomotion/run/Q_z", Q_z);
  ros::param::get("ConvexMPCLocomotion/run/Q_droll", Q_droll);
  ros::param::get("ConvexMPCLocomotion/run/Q_dpitch", Q_dpitch);
  ros::param::get("ConvexMPCLocomotion/run/Q_dyaw", Q_dyaw);
  ros::param::get("ConvexMPCLocomotion/run/Q_dx", Q_dx);
  ros::param::get("ConvexMPCLocomotion/run/Q_dy", Q_dy);
  ros::param::get("ConvexMPCLocomotion/run/Q_dz", Q_dz);
  ros::param::get("ConvexMPCLocomotion/run/swing_hight", swing_hight);
  ros::param::get("ConvexMPCLocomotion/run/first_hight", first_hight);
  
}

/******************************************************************************************************/
/******************************************************************************************************/

void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;

  auto &seResult = data._stateEstimator->getResult();
  // 返回StateEstimatorData.result变量，
  // 该变量在OrientationEstimator.cpp的CheaterOrientationEstimator::run()进行更新
  auto &stateCommand = data._desiredStateCommand;

  // pick gait
  Gait *gait = &standing;
  if (gaitNumber == 1)
    gait = &standing;
  else if (gaitNumber == 2)
    gait = &walking;

  current_gait = gaitNumber;
  // integrate position setpoint
  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0);
  // 设置目标速度，stateCommand->data.stateDes在DesiredCommand::setStateCommands中更新，
  // 该函数在FSMState_Walking::run中调用

  // 机器人相对于世界的速度
  Vec3<double> v_des_world;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  // rBody乘v_des_robot得到机器人相对于世界的速度
  Vec3<double> v_robot = seResult.vWorld;
  // seResult.vWorld在PositonVelocityEstimator.cpp的CheaterPositionVelocityEstimator::run()进行更新
  // 数据源自于CheatIO::StateCallback，从gazebo得到关于机器人的线速度msg
  // 在实体机中，这个线速度，需要一个里程计，简单操作是对imu的加速度进行积分然后使用卡尔曼进行观测
  // 复杂操作是从LIO或VIO拿到里程计数据
  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = hight;
  // 这是对机器人的质心的高度的需求，是机器人初始状态腿部弯曲且静止的高度

  // get then foot location in world frame
  for (int i = 0; i < 2; i++)
  {
    pFoot[i] = seResult.position +
               // seResult.position是在PositonVelocityEstimator.cpp的CheaterPositionVelocityEstimator::run()进行更新，
               // 数据源自于CheatIO::StateCallback，从gazebo得到关于机器人的位置msg
               // 实际应该是baselink相对于世界坐标系的位置
               // 同seResult.vWorld，在实机中应该从VIO或LIO获取
               seResult.rBody.transpose() * (data._biped->getHip2Location(i) + data._legController->data[i].p);
    // data._biped->getHip2Location(i)返回的是Hip2关节的偏移向量，函数在Biped.h中定义
    // 函数用到的leg_offset_**需要根据以质心坐标系为参考，质心到hip2关节的偏移来填写。
    // 假如质心就是baselink，而hip到baselink在x方向没有偏移，并且calf和thigh关节轴线与hip轴线共面
    // 那么leg_offset_x2=0.0，那么 leg_offset_y2 为y轴baselink到hip2关节轴线的距离，leg_offset_z2同理
    // data._legController->data[i].p 来自于 LegController.h 的LegControllerData结构体
    // p是三轴位置
    // pFoot是计算髋关节到世界坐标系的位置
    std::cout << "Foot" << i << " " << pFoot[i] << "  " << std::endl;
  }
  std::cout << std::endl;
  // some first time initialization
  if (firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0, 0, 0); // connect to desired state command later
    Vec3<double> v_des_world(0, 0, 0); // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    // pBody_des是机器人目标位置
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    // vBody_des是机器人目标线速度
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    // pBody_RPY_des和vBody_Ori_des貌似并没有被用到
    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    //
    if (gaitNumber == 1)
    // 如果步态是站立步态，那么baselink目标位置为0.55
    // 这个值要根据机器人来适配
    // 看起来整个鄂判断并不会进取，因为修改gaitNumber的代码只在 FSMState_Walking.cpp的FSMState_Walking::run()中
    // 调用了setGaitNum(2)
    {
      pBody_des[0] = seResult.position[0];
      pBody_des[1] = seResult.position[1];
      pBody_des[2] = hight;
      vBody_des[0] = 0;
      vBody_des[0] = 0;
    }

    for (int i = 0; i < 2; i++)
    {
      footSwingTrajectories[i].setHeight(first_hight);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }

  contact_state = gait->getContactSubPhase(); // 更新接触脚的步态，返回值并没有用到

  // foot placement
  swingTimes[0] = dtMPC * gait->_swing;
  swingTimes[1] = dtMPC * gait->_swing;

  double side_sign[2] = {1, -1};
  double interleave_y[2] = {-0.1, 0.1};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  // 用于控制一个双足机器人步态的一部分，其中涉及到腿部摆动的轨迹规划。
  // 代码的主要功能是计算每只脚在摆动过程中的剩余时间，
  // 并根据当前的状态和期望的速度来更新脚的最终位置。
  for (int i = 0; i < 2; i++) // 循环分别处理机器人的两只脚
  {
    // firstSwing的更新见 转阅处No.1，这是一个首次进入的标志位
    // 这里判断每只脚是否处于摆动的第一个周期。
    // 如果是，就将剩余摆动时间设置为预设的摆动时间；
    // 如果不是，就从剩余时间中减去时间步长dt
    // swingTimeRemaining的使用见 转阅处No.2
    // swingTimeRemaining首先是swingTimes，然后-=dt，逐步减小
    if (firstSwing[i])
      swingTimeRemaining[i] = swingTimes[i];
    else
      swingTimeRemaining[i] -= dt;

    footSwingTrajectories[i].setHeight(swing_hight); // 每只脚设置摆动轨迹的高度，这里设置为0.1米。
    Vec3<double> offset(0, side_sign[i] * 0.0, -0.0);
    // 看起来offset并不起作用
    Vec3<double> pRobotFrame = (data._biped->getHip2Location(i) + offset);
    // data._biped->getHip2Location(i)返回的是Hip2关节的偏移向量，函数在Biped.h中定义
    // pRobotFrame的使用见 转阅处No.2，数据本身代表的是Hip2关节的偏移向量

    Vec3<double> des_vel;
    des_vel[0] = stateCommand->data.stateDes(6);
    des_vel[1] = stateCommand->data.stateDes(7);
    des_vel[2] = stateCommand->data.stateDes(8);
    // 获取期望的速度向量

    // 转阅处No.2
    Vec3<double> Pf = seResult.position +
                      // seResult.position是在PositonVelocityEstimator.cpp的CheaterPositionVelocityEstimator::run()进行更新，
                      // 数据源自于CheatIO::StateCallback，从gazebo得到关于机器人的位置msg
                      // 实际应该是baselink相对于世界坐标系的位置
                      // 同seResult.vWorld，在实机中应该从VIO或LIO获取
                      seResult.rBody.transpose() * pRobotFrame + seResult.vWorld * swingTimeRemaining[i];
    // rBody*pRobotFrame得到质心相对于世界的位置
    // vWorld * swingTimeRemaining 根据当前 质心相对于世界的速度计算出剩下的时间里会移动的距离

    // ？？？？？？？？pfx_rel和pfy_rel的计算是什么意思
    double p_rel_max = 0.4; // 设置一个相对位置的最大值
    double pfx_rel = -0.01 + 
                    seResult.vWorld[0] * 0.5 * gait->_stance * dtMPC +
                     0.02 * (seResult.vWorld[0] - v_des_world[0]);

    double pfy_rel = seResult.vWorld[1] * 0.5 * gait->_stance * dtMPC +
                     0.02 * (seResult.vWorld[1] - v_des_world[1]);
    // printf("foot pos rel:%lf  %lf\n",pfx_rel,pfy_rel);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    // 脚的相对位置调整限制在最大值范围内
     std:: cout << "pfy_rel =" << pfy_rel << "\n";
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel; //+ interleave_y[i] * v_abs * interleave_gain;
    Pf[2] = -0.0;
    // 更新脚的最终位置，包括相对位置调整
    footSwingTrajectories[i].setFinalPosition(Pf);
  }

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter); // 步态进行迭代，设置步态迭代的次数和当前迭代计数

  // load LCM leg swing gains
  // Kp << 300, 0, 0,
  //     0, 300, 0,
  //     0, 0, 300;
  Kp << Kp_param, 0, 0,
      0, Kp_param, 0,
      0, 0, Kp_param;
  Kp_stance = 0 * Kp;

  // Kd << 10, 0, 0,
  //     0, 10, 0,
  //     0, 0, 10;
  Kd << Kd_param, 0, 0,
      0, Kd_param, 0,
      0, 0, Kd_param;
  Kd_stance = 0 * Kd;
  // 设置腿部摆动时的比例增益（Kp）和导数增益（Kd），
  // 以及站立相时的增益（这里站立相的增益被设置为0）

  Vec2<double> contactStates = gait->getContactSubPhase();
  Vec2<double> swingStates = gait->getSwingSubPhase();
  // 获取每只脚的接触和摆动子阶段状态。

  int *mpcTable = gait->mpc_gait();
  // 获取MPC步态表的指针
  updateMPCIfNeeded(mpcTable, data, omniMode);
  // 根据MPC步态表和其他数据更新MPC
  iterationCounter++;
  // 迭代计数器递增
  Vec2<double> se_contactState(0, 0);
  // se_contactState其实数据是contactStates
  // 初始化接触状态向量
  for (int foot = 0; foot < 2; foot++)
  {

    double contactState = contactStates(foot);
    double swingState = swingStates(foot);
    // 获取当前脚的接触状态和摆动状态
    std::cout << "swing " << foot << ": " << swingState << std::endl;
    std::cout << "Contact " << foot << ": " << contactState << std::endl;
    Vec3<double> pFootWorld;

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot]) // 转阅处No.1
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      } // 设置初始位置并重置标志

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      // 计算脚的摆动轨迹
      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      // 获取脚的期望世界坐标位置和速度
      double side = -1.0;
      if (foot == 1)
      {
        side = 1.0;
      }
      Vec3<double> hipOffset = {0, -side * data._biped->leg_offset_y2, 0.};
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - hipOffset;
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // 计算脚相对于机器人身体的期望位置和速度
      if (vDesLeg.hasNaN())
      {
        vDesLeg << 0, 0, 0;
      }

      data._legController->commands[foot].feedforwardForce << 0, 0, 0, 0, 0, 0;
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp;
      data._legController->commands[foot].kdCartesian = Kd;
      data._legController->commands[foot].kptoe = Kptoe_param; // 0
      data._legController->commands[foot].kdtoe = Kdtoe_param;
      // 设置腿部控制器的期望位置、速度和控制增益
      se_contactState[foot] = contactState;
    }

    else if (contactState > 0) // foot is in stance
    // 脚处于站立状态，执行类似的操作，但使用站立相的控制增益。
    {
      firstSwing[foot] = true;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._biped->getHip2Location(foot);
      Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      if (vDesLeg.hasNaN())
      {
        vDesLeg << 0, 0, 0;
      }

      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp_stance; // 0
      data._legController->commands[foot].kdCartesian = Kd_stance;

      data._legController->commands[foot].kptoe = 0; // 0
      data._legController->commands[foot].kdtoe = 0;

      data._legController->commands[foot].feedforwardForce = f_ff[foot];

      se_contactState[foot] = contactState;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData &data, bool omniMode)
{

  if ((iterationCounter % 5) == 0) // 每5次迭代更新一次
  {
    // 获取状态估计器的结果和期望的状态命令
    auto seResult = data._stateEstimator->getResult();
    auto &stateCommand = data._desiredStateCommand;
    // 位置、世界坐标系下的速度、角速度和四元数表示的姿态
    double *p = seResult.position.data();
    double *v = seResult.vWorld.data();
    double *w = seResult.omegaWorld.data();
    double *quat = seResult.orientation.data();

    // Joint angles to compute foot rotation
    Eigen::Matrix<double, 10, 1> q;
    for (int i = 0; i < 2; i++)
    {
      for (int k = 0; k < 5; k++)
      {
        q(i * 5 + k) = data._legController->data[i].q(k);
      } // 获取机器人腿部关节的角度
    }

    double *joint_angles = q.data();

    double PI = 3.14159265359;
    // Joint angles offset correction
    // 对关节角度进行偏移校正，因为在urdf中添加了旋转偏置，这里要补偿上
    // ???????但urdf里是0.25/-0.5/0.25
    q(2) -= 0.25 * PI;
    q(3) += 0.5 * PI;
    q(4) -= 0.25 * PI;

    q(7) -= 0.25 * PI;
    q(8) += 0.5 * PI;
    q(9) -= 0.25 * PI;

    double PI2 = 2 * PI;
    for (int i = 0; i < 10; i++)
    {
      q(i) = fmod(q(i), PI2);
    } // 将关节角度限制在[0, 2π]的范围内

    double r[6];
    for (int i = 0; i < 6; i++)
    {
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
    } // 计算脚部位置相对于机器人身体的位置
    
    // MPC Weights
    double Q[12] = {Q_roll, Q_pitch, Q_yaw,    // roll pitch yaw
                    Q_x, Q_y, Q_z,             // x y z
                    Q_droll, Q_dpitch, Q_dyaw, // droll dpitch dyaw
                    Q_dx, Q_dy, Q_dz};         // dx dy dz
    // double Q[12] = {100, 100, 150, 200, 200, 300, 1, 1, 1, 1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    double Alpha[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

    double *weights = Q;
    double *Alpha_K = Alpha;
    // MPC的权重和正则化项
    double yaw = seResult.rpy[2];
    // 获取当前的偏航角
    std::cout << "current position: \n"
              << p[0] << "  " << p[1] << "  " << p[2] << std::endl;
    std::cout << "current rpy:\n"
              << seResult.rpy.transpose() << std::endl;
    v_des_robot << stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0;
    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    // 期望的机器人速度和世界坐标系下的速度
    const double max_pos_error = .05; // 最大位置误差
    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];

    if (xStart - p[0] > max_pos_error)
      xStart = p[0] + max_pos_error;
    if (p[0] - xStart > max_pos_error)
      xStart = p[0] - max_pos_error;

    if (yStart - p[1] > max_pos_error)
      yStart = p[1] + max_pos_error;
    if (p[1] - yStart > max_pos_error)
      yStart = p[1] - max_pos_error;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    // Vec3<double> ori_des_world;
    ori_des_world << stateCommand->data.stateDes[3], stateCommand->data.stateDes[4], stateCommand->data.stateDes[5];
    // 期望的姿态

    double trajInitial[12] = {/*rpy_comp[0] + */ stateCommand->data.stateDes[3], // 0 //MPC的初始轨迹
                              /*rpy_comp[1] + */ stateCommand->data.stateDes[4], // 1
                              seResult.rpy[2],                                   // 2
                              xStart,                                            // 3
                              yStart,                                            // 4
                              hight,                                             // 5  ？？？？？？？？是否需要修改
                              0,                                                 // 6
                              0,                                                 // 7
                              stateCommand->data.stateDes[11],                   // 8
                              v_des_world[0],                                    // 9
                              v_des_world[1],                                    // 10
                              0};                                                // 11
    for (int i = 0; i < horizonLength; i++)                                      // 设置MPC的预测轨迹
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];

      if (i == 0) // start at current position  TODO consider not doing this
      {
        trajAll[0] = seResult.rpy[0];
        trajAll[1] = seResult.rpy[1];
        trajAll[2] = seResult.rpy[2];
        trajAll[3] = seResult.position[0];
        trajAll[4] = seResult.position[1];
        trajAll[5] = seResult.position[2];
      }
      else
      {
        if (v_des_world[0] == 0)
        {
          trajAll[12 * i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
        }
        else
        {
          trajAll[12 * i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0];
        }
        if (v_des_world[1] == 0)
        {
          trajAll[12 * i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
        }
        else
        {
          trajAll[12 * i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1];
        }
        if (stateCommand->data.stateDes[11] == 0)
        {
          trajAll[12 * i + 4] = trajInitial[4];
        }
        else
        {
          trajAll[12 * i + 2] = yaw + i * dtMPC * stateCommand->data.stateDes[11];
          // std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      }
      // std::cout << "traj " << i << std::endl;
      // for (int j = 0; j < 12; j++)
      // {
      //   std::cout << trajAll[12 * i + j] << "  ";
      // }
      // std::cout << " " << std::endl;
    }

    // MPC Solver Setup
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC, horizonLength, 0.25, 500);

    // Solve MPC
    Timer t_mpc_solve;
    t_mpc_solve.start();
    update_problem_data(p, v, quat, w, r, joint_angles, yaw, weights, trajAll, Alpha_K, mpcTable);
    // printf("MPC Solve time %f ms\n", t_mpc_solve.getMs());

    // Get solution and update foot forces
    for (int leg = 0; leg < 2; leg++)
    {
      Vec3<double> GRF;
      Vec3<double> GRF_R;
      Vec3<double> GRM;
      Vec3<double> GRM_R;
      Vec6<double> f;
      for (int axis = 0; axis < 3; axis++)
      {
        GRF[axis] = get_solution(leg * 3 + axis);
        GRM[axis] = get_solution(leg * 3 + axis + 6);
      }
      GRF_R = -seResult.rBody * GRF;
      GRM_R = -seResult.rBody * GRM;
      // std::cout << "RBody: " << seResult.rBody << std::endl;

      for (int i = 0; i < 3; i++)
      {
        f(i) = GRF_R(i);
        f(i + 3) = GRM_R(i);
      }
      f_ff[leg] = f;
    }
  }
}