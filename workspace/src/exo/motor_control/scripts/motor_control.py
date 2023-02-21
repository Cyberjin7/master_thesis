#!/usr/bin/env python
import pigpio
import time
import rospy
import rospkg
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
import subprocess

import math

class MotorElbow:

    def __init__(self):
        self.pi = pigpio.pi()
        self.pin_output = 18 #pwm pin
        
        #self.min_elbow_angle = 0.5235987755982988
        self.min_elbow_angle = 0
        self.last_angle = self.min_elbow_angle
        self.max_elbow_angle = 2.0943951023931953

        self.servo_pos1 = 2500 #2500
        self.servo_pos2 = 500 
        self.last_pos = self.servo_pos1

        self.pi.set_servo_pulsewidth(self.pin_output,0)
        self.servo_attached = True

        self.motor_setup()
        
        self.desired_angle_sub = rospy.Subscriber('/q_control_publisher', Float64, self.callback_set_motor_angle, queue_size=1, buff_size=64*10) 
        self.attach_motor_srv = rospy.Service('/q_control_service', SetBool, self.callback_attach_motor)

    def motor_setup(self):
        #define output pin for PWM 
        self.pi.set_mode(self.pin_output, pigpio.OUTPUT)
        print (f'mode {self.pin_output}: {self.pi.get_mode(self.pin_output)}')

        self.pi.set_servo_pulsewidth(self.pin_output,self.last_pos)
        rospy.sleep(2)

    # move between servo_pos1 and servo_pos2 at 1 Hz rate
    def motor_loop(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            #self.pi.set_servo_pulsewidth(self.pin_output,self.last_pos)

            #if(self.last_pos == self.servo_pos1):
            #    self.last_pos = self.servo_pos2
            #else:
            #    self.last_pos = self.servo_pos1

            rate.sleep()


    #def map_numbers(self,x, in_min, in_max, out_min, out_max):
    #    return ((self.check_limits(x) - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    def map_numbers(self,x):
        return ((self.check_limits(x) - 10) * (-2500 + 500) / (100 - 10) + 2500)

    def check_limits(self, input):
        # check if the input is within the elbow ROM
        #if input > math.degrees(self.max_elbow_angle):
        #    input = math.degrees(self.max_elbow_angle)
        #if input < math.degrees(self.min_elbow_angle):
        #    input = math.degrees(self.min_elbow_angle)
        # check if the input is within the servo motor (with gearhead) ROM
        #if input > 120:
        #    input = 120
        #if input < 30:
        #    input = 30
        if input > 100:
            input = 100
        if input < 10:
            input = 10
        return input

    def callback_set_motor_angle(self, msg):
        if self.servo_attached == True:
            #self.pi.set_servo_pulsewidth(self.pin_output, self.map_numbers(msg.data, 30, 120, 500, 2500))
            self.pi.set_servo_pulsewidth(self.pin_output, self.map_numbers(msg.data))
            #print("send command to the servo motor with the angle input: " + str(self.check_limits(msg.data)))
        
    def callback_attach_motor(self, req):
        self.servo_attached = req.data
        print("servo_attached: " + str(self.servo_attached))
        return [self.servo_attached, '']
        
if __name__ == '__main__':
    #init ros node
    rospy.init_node('motor_control')

    try:
        bashCommand = "sudo killall pigpiod" 
        output = subprocess.check_output(['bash','-c', bashCommand]) 
        print(output)
        rospy.sleep(1) # sleep for 1 s
        rospy.logwarn("Kill old pigpiod")
    except:
        print("No old pigpiod instances")

    try:
        bashCommand = "sudo pigpiod" 
        output = subprocess.check_output(['bash','-c', bashCommand]) 
        print(output)
        rospy.sleep(1) # sleep for 1 s
        rospy.logwarn("Init pigpiod")
    except:
        rospy.logerr("pigpiod init failed")

    motor = MotorElbow()
    motor.motor_loop()
