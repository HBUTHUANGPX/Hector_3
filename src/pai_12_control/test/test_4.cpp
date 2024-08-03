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
    LegController *leg = new LegController(biped);
    IOInterface *ioInter;
    ioInter = new CheatIO(robot_name);
    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    signal(SIGINT, ShutDown);
    for (int leg = 0; leg < 2; leg++)
    {
        for (int j = 0; j < 5; j++)
        {
            state->motorState[leg * 5 + j].q = 0.0;
            state->motorState[leg * 5 + j].dq = 0.0;
            state->motorState[leg * 5 + j].tauEst = 0.0;
        }
    }

    while (running)
    {
        leg->updateData(state);
        break;
        rate.sleep();
    }
    system("stty sane");  //Terminal back to normal
    delete ioInter;
    delete cmd;
    delete state;
    return 0;
}
