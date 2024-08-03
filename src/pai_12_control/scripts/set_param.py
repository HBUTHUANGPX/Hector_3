#! /usr/bin/env python

import rospy

if __name__ == "__main__":
    rospy.init_node("set_update_paramter_p")

    rospy.set_param("ConvexMPCLocomotion/run/p_rel_max",0.4)
    rospy.set_param("ConvexMPCLocomotion/run/Kp",200)
    rospy.set_param("ConvexMPCLocomotion/run/Kd",5)
    rospy.set_param("ConvexMPCLocomotion/run/kptoe",1)
    rospy.set_param("ConvexMPCLocomotion/run/kdtoe",0.01)
    rospy.set_param("ConvexMPCLocomotion/updateMPCIfNeeded/Q",[100,100,150,200,200,300,1,1,1,1,1,1])


    rospy.set_param("LegController/updateCommand/kphip0",0.5)
    rospy.set_param("LegController/updateCommand/kdhip0",0.1)

    rospy.set_param("SolverMPC/solve_mpc/lt",0.04)
    rospy.set_param("SolverMPC/solve_mpc/lh",0.05)

    rospy.set_param("Constraints/CalculateConstarintMatrix/lt",0.04)
    rospy.set_param("Constraints/CalculateConstarintMatrix/lh",0.05)

    

    



    
