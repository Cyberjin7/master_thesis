#! /usr/bin/ev python

import rospy
from std_msgs.msg import Float64
from custom_ros_msgs.msg import CustomData
import numpy as np



if __name__ == '__main__':
    pub = rospy.Publisher('sine', CustomData, queue_size=100)
    rospy.init_node('sine_publisher', anonymous=True)
    rate = rospy.Rate(200)

    sine = CustomData()

    start_time = rospy.get_rostime()

    while not rospy.is_shutdown():
        ros_time = rospy.get_rostime() - start_time
        sine.header.stamp = ros_time
        sine.value.data = np.sin(ros_time.to_sec())
        pub.publish(sine)
        rate.sleep()
