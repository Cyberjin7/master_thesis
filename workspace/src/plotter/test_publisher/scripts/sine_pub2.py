#! /usr/bin/ev python

import rospy
from std_msgs.msg import Float64
from custom_ros_msgs.msg import CustomData
import numpy as np



if __name__ == '__main__':
    pub = rospy.Publisher('sine2', CustomData, queue_size=100)
    rospy.init_node('sine_publisher2', anonymous=True)
    rate = rospy.Rate(20)

    sine = CustomData()
    start_time = rospy.get_rostime()

    delay = rospy.get_param("~delay")
    rospy.loginfo("Delay is %s seconds.", delay)

    while not rospy.is_shutdown():

        ros_time = rospy.get_rostime() - start_time
        ros_time.secs += delay
        sine.value.data = np.sin(ros_time.to_sec())*2
        sine.header.stamp = ros_time
        pub.publish(sine)
        rate.sleep()
