#! /usr/bin/env python

import rospy
from unitree_legged_msgs.msg import MotorCmd
from unitree_legged_msgs.msg import MotorState
first_name = "hector_gazebo"
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from  gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
import numpy as np

class motor:
    def __init__(self,side = "L", joint = "hip"):
        self.side = side
        self.joint = joint
        self.cmd= MotorCmd()
        self.sta= MotorState()
        self.cmd_sub = rospy.Subscriber(first_name+"/"+side+"_"+joint+"_controller/command",MotorCmd,self.do_cmd_sub,queue_size=1)
        self.sta_sub = rospy.Subscriber(first_name+"/"+side+"_"+joint+"_controller/state",MotorState,self.do_sta_sub,queue_size=1)
    def do_cmd_sub(self,cmd = MotorCmd()):
        # print(self.side+"_"+self.joint+"cmd")
        self.cmd = cmd
    def do_sta_sub(self,sta = MotorState()):
        # print(self.side+"_"+self.joint+"sta")
        self.sta = sta
class body:
    def __init__(self):
        self.pose = Pose()
        self.twist = Twist()
        self.body_sub = rospy.Subscriber("gazebo/model_states",ModelStates,self.cb,queue_size=1)
        self.uservalue_sub = rospy.Subscriber("user_value",Float32MultiArray,self.uservalue_cb,queue_size=1)
        self.F_M_sub = rospy.Subscriber("F_M",Float32MultiArray,self.F_M_cb,queue_size=1)
        self.uservalue = Float32MultiArray();
        ma = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],dtype=np.float32)
        self.uservalue.data = ma.flatten().tolist()
        self.F_M = Float32MultiArray();
        ma = np.array([0.0,0.0,0.0,0.0,0.0,0.0],dtype=np.float32)
        self.F_M.data = ma.flatten().tolist()
    
    def cb(self,ms = ModelStates()):
        if len(ms.name) == 3:
            self.pose = ms.pose[2]
            self.twist = ms.twist[2]
        else:
            print(len(ms.name))
        ...
    def uservalue_cb(self,arr = Float32MultiArray()):
        self.uservalue = arr
        ...
    def F_M_cb(self,arr = Float32MultiArray()):
        self.F_M = arr
        ...
if __name__ == "__main__":
    rospy.init_node("haha")

    rate = rospy.Rate(1000)
    motors = []
    side = ["L","R"]
    joint = ["hip","hip2","thigh","calf","toe"]
    for _side in side:
        for _joint in joint:
            motors.append(motor(_side,_joint))
    print(len(motors))
    m = motor()
    b = body()
    with open('number.txt','w') as file:
        while not rospy.is_shutdown():

            # 20
            for i in range(2):
                for j in range(5):
                    m = motors[i*5+j]
                    s = str(m.cmd.tau)+","+str(m.sta.tauEst)+","
                    print(s,end="")
                    file.write(s)
            # 13
            s = ""
            s = str(b.twist.linear.x)+","+str(b.twist.linear.y)+","+str(b.twist.linear.z)+","
            s = s + str(b.twist.angular.x)+","+str(b.twist.angular.y)+","+str(b.twist.angular.z)+","
            s = s + str(b.pose.orientation.w)+","+str(b.pose.orientation.x)+","+str(b.pose.orientation.y)+","+str(b.pose.orientation.z)+","
            s = s + str(b.pose.position.x)+","+str(b.pose.position.y)+","+str(b.pose.position.z)+","
            print(s,end = "")
            file.write(s)

            # 11
            s = ""
            s = str(b.F_M.data[0])+","+str(b.F_M.data[1])+","+str(b.F_M.data[2])+","+str(b.F_M.data[3])+","+str(b.F_M.data[4])+","+str(b.F_M.data[5])+","
            s = s + str(b.uservalue.data[0])+","+str(b.uservalue.data[1])+","+str(b.uservalue.data[2])+","+str(b.uservalue.data[3])+","+str(b.uservalue.data[4])
            file.write(s)
            print(s,end = "")
            print("")
            file.write('\n') 
            rate.sleep()
    ...