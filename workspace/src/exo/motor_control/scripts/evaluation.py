#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import subprocess
import math as math


class Evaluation: 
    def __init__(self):
        self.MVC = 0.5

        self.q_intention = 0
        rospy.wait_for_message('/q_intention', Float64)
        rospy.Subscriber('/q_intention', Float64, self.callbackSetIntention)
        self.q_state = 0
        rospy.wait_for_message('/q_intention', Float64)
        rospy.Subscriber('/q_intention', Float64, self.callbackSetState)

        self.emg_state = 0
        rospy.Subscriber('/emg_imu_data', Float64MultiArray, self.callbackSetEMG)

        self.error = 0

    def callbackSetIntention(self,msg):
        self.q_intention = msg.data

    def callbackSetState(self,msg):
        self.q_state = msg.data

    def callbackSetEMG(self,msg):
        self.emg_state = msg.data[0]/self.MVC

    def absError(self):
        self.error = abs(self.q_intention-self.q_state)/self.q_intention

    def rosLoop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.absError()
            print(self.error)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('evaluation')

    eval = Evaluation()
    eval.rosLoop()


