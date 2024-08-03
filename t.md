# 代码函数文件结构

```src/main.cpp->main()->_FSMController->run()```

```FSM.cpp->FSM::run()->_data->sendRecv->ControlFSMData.h->sendRecv()->_interface->sendRecv(_lowCmd, _lowState)->CheatIO::sendRecv()```

```FSM.cpp->FSM::run()->_currentState->run()```

MPC:
```FSMState_Walking.cpp->FSMState_Walking::run()->Cmpc.run(*_data);```

```ConvexMPCLocomotion.cpp->ConvexMPCLocomotion::run->ConvexMPCLocomotion::updateMPCIfNeeded->```

```convexMPC_interface.cpp->setup_problem() && update_problem_data()->solve_mpc()```

```SolveMPC.cpp->solve_mpc()```

需要修改的地方

1. ConvexMPCLocomotion.cpp 
   
   world_position_desired[2] = 0.55

   pBody_des[2] = 0.55

   hipOffset = {0, side * -0.02, -0.136}

    q(2) += 0.3 * PI;

    q(3) -= 0.6 * PI;

    q(4) += 0.3 * PI;

    q(7) += 0.3 * PI;

    q(8) -= 0.6 * PI;
    
    q(9) += 0.3 * PI;

    double trajInitial[12]的0.55

2. Biped.h
   
    leg_offset_x2,leg_offset_y2,leg_offset_z2

3. LegController.cpp

    q(2) = q(2) + 0.3*3.14159;

    q(3) = q(3) - 0.6*3.14159;

    q(4) = q(4) + 0.3*3.14159;

4. RobotState.cpp

    Id << 0.5413, 0.5200, 0.0691;

5. SolverMPC.cpp

    q(2) +=  0.3*PI;
    q(3) -=  0.6*PI;
    q(4) +=  0.3*PI;

    q(7) +=  0.3*PI;
    q(8) -=  0.6*PI;
    q(9) +=  0.3*PI;

    double motorTorqueLimit = 33.5