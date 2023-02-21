#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool

servo_attached = False

def servo_attach_callback(req):
    servo_attached = req.data
    print("servo_attached: " + str(servo_attached))
    return [servo_attached, '']

def servo_attach_server():
    rospy.init_node('servo_attach_server', anonymous = True)
    s = rospy.Service('q_control_service', SetBool, servo_attach_callback)
    print("Ready")
    rospy.spin()

if __name__ == "__main__":
    servo_attach_server()