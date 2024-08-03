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

bool running = true;
void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
    system("stty sane"); // Terminal back to normal
    exit(0);
}

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
    int j = 0;
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
    double derta = 0.0001*3.1415926;
    double target_thigh_L = 0.0;
    double target_thigh_R = 0.0;
    double target_calf_L = 0.0;
    double target_calf_R = 0.0;
    double target_toe_R = 0.0;
    double target_toe_L = 0.0;
    while (running)
    {
        target_thigh_L-=derta;
        target_thigh_R-=derta;
        target_calf_L+=2*derta;
        target_calf_R+=2*derta;
        target_toe_L-=derta;
        target_toe_R-=derta;
        j++;
        // std::cout << "===============new===" << j++ << "===================" << std::endl;
        cmd->motorCmd[0].tau = (0 - state->motorState[0].q) * Kp[0] + (0 - state->motorState[0].dq) * Kd[0];
        cmd->motorCmd[1].tau = (0 - state->motorState[1].q) * Kp[1] + (0 - state->motorState[1].dq) * Kd[1];
        cmd->motorCmd[2].tau = (target_thigh_L - state->motorState[2].q) * Kp[2] + (0 - state->motorState[2].dq) * Kd[2];
        cmd->motorCmd[3].tau = (target_calf_L - state->motorState[3].q) * Kp[3] + (0 - state->motorState[3].dq) * Kd[3];
        cmd->motorCmd[4].tau = (target_toe_L - state->motorState[4].q) * Kp[4] + (0 - state->motorState[4].dq) * Kd[4];
        std::cout<<target_toe_L<<"  "<<state->motorState[4].q<<"  ";
        cmd->motorCmd[5].tau = (0 - state->motorState[5].q) * Kp[5] + (0 - state->motorState[5].dq) * Kd[5];
        cmd->motorCmd[6].tau = (0 - state->motorState[6].q) * Kp[6] + (0 - state->motorState[6].dq) * Kd[6];
        cmd->motorCmd[7].tau = (target_thigh_R - state->motorState[7].q) * Kp[7] + (0 - state->motorState[7].dq) * Kd[7];
        cmd->motorCmd[8].tau = (target_calf_R - state->motorState[8].q) * Kp[8] + (0 - state->motorState[8].dq) * Kd[8];
        cmd->motorCmd[9].tau = (target_toe_R - state->motorState[9].q) * Kp[9] + (0 - state->motorState[9].dq) * Kd[9];
        if (j==2000)
        {
            derta*=-1;
            j=0;
        }
        else
        {
            // std::cout<<j<<"   ";
        }
        
        ioInter->sendRecv(cmd, state);
        rate.sleep();
    }

    // system("stty sane");  //Terminal back to normal
    delete ioInter;
    delete cmd;
    delete state;
    return 0;
}
