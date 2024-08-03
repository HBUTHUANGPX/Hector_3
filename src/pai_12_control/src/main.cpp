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

int main(int argc, char ** argv)
{
    IOInterface *ioInter;
    ros::init(argc, argv, "pai_12_control", ros::init_options::AnonymousName);
    
    std::string robot_name = "hector";
    std::cout << "robot name " << robot_name << std::endl;

    ioInter = new CheatIO(robot_name);
    ros::Rate rate(1000);

    double dt = 0.001;
    Biped biped;
    biped.setBiped();

    LegController* legController = new LegController(biped);
    LowlevelCmd* cmd = new LowlevelCmd();
    LowlevelState* state = new LowlevelState();

    std::cout << "start setup " << std::endl;
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<CheaterOrientationEstimator>();   
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();   

    std::cout << "setup state etimator" << std::endl;                                                             

    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &biped;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

    FSM* _FSMController = new FSM(_controlData);

    signal(SIGINT, ShutDown);
    
    // while (ros::ok())
    // {//先蹲下
        
    // }
    
    while(running)
    {
        std::cout<<"===============new==================="<<std::endl;
        _FSMController->run();
        rate.sleep();
    }
    
    // system("stty sane");  //Terminal back to normal
    delete _controlData;
    return 0;

}
