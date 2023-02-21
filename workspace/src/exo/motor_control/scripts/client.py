#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool

def get_bool():
    while True:
        try:
           return {"true":True,"false":False}[input("Do you want to attach the servo? please enter True or False: ").lower()]
        except KeyError:
           print("Invalid input: please enter True or False!")

def servo_attach_client():
    rospy.wait_for_service('q_control_service')
    try:
        servo_attach_servie = rospy.ServiceProxy('q_control_service', SetBool)
        resp1 = servo_attach_servie(get_bool())
        print("servo_attached: " + str(resp1.success))
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    servo_attach_client()