#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/CheatIO.h"
#include "../include/FSM/FSM.h"

#include "../include/jocobia/calculate_jocobia.h"
#include "../include/jocobia/F_M_calculate.h"
bool running = true;
double PI = 3.14159265359;
void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
    system("stty sane"); // Terminal back to normal
    exit(0);
}
struct LegData
{
    LegData() {}
    Vec5<double> q, qd, tau; // q是关节角度 qd是关节速度
    Vec3<double> p, v;       // p是位置，v是速度
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_1");
    ros::NodeHandle n;
    std::string robot_name = "hector";

    ros::Rate rate(1000);
    double dt = 0.001;
    std::cout << "robot name " << robot_name << std::endl;
    Biped biped;
    biped.setBiped();
    IOInterface *ioInter;
    ioInter = new CheatIO(robot_name);
    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    signal(SIGINT, ShutDown);

    {
        double Kp[10] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        for (size_t i = 0; i < 10; i++)
        {
            Kp[i] = 10.0;
        }
        double Kd[10] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        for (size_t i = 0; i < 10; i++)
        {
            Kd[i] = 0.1;
        }
        double derta = 0.0001 * 3.1415926;
    }
    int j = 0;
    calculate_jocobia a;
    LegData data[2];
    Vec6<double> F_M;
    F_M[0] = 0.0;
    F_M[1] = 0.0;
    F_M[2] = 0.0;
    F_M[3] = 0.0;
    F_M[4] = 0.0;
    F_M[5] = 0.0;
    double derta = 0.001 * PI;
    // a.print_jointPlacements();
    a.print_joint_id_with_name();
    // a.print_link_id_with_name();
    ros::Rate r(100);
    F_M_calculate c;
    // a.my_calculate_jocobia();
    std::cout<<asin(1/sqrt(2))/M_PI*180<<std::endl;
    while (1)
    {
        // ioInter->sendRecv(cmd, state);
        // std::cout<<cos(PI)<<std::endl;
        for (size_t j = 0; j < 1; j++)
        {
            for (int leg = 0; leg < 2; leg++)
            {
                for (int j = 0; j < 5; j++)
                {
                    data[leg].q(j) = 0.0;
                    data[leg].qd(j) = 0.0;
                    data[leg].tau(j) = 0.0;
                }
                data[leg].q(2) += derta * j;
                data[leg].q(3) -= derta * 2 * j;
                data[leg].q(4) += derta * j;
                F_M = c.calculate(data[leg].q(2));
                // data[leg].q(2) -= 0.78;
                // data[leg].q(3) += 1.56;
                // data[leg].q(4) -= 0.78;
                a.fresh_joint(leg, data[leg].q);
                a.calculat_leg_J(leg);
                // a.calculate_endpoint_vel_and_ang_vel(leg, data[leg].qd);
                a.calculate_joint_torque_3(leg, F_M);
            }
            r.sleep();
        }
        break;
        rate.sleep();
    }

    system("stty sane"); // Terminal back to normal
    delete ioInter;
    delete cmd;
    delete state;
    return 0;
}
