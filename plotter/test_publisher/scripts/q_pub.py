#! /usr/bin/ev python

import rospy
from std_msgs.msg import Float64
from custom_ros_msgs.msg import CustomData
import numpy as np



if __name__ == '__main__':
    pub = rospy.Publisher('q', Float64, queue_size=100)
    rospy.init_node('q_publisher', anonymous=True)
    rate = rospy.Rate(20)

    q = Float64()

    while not rospy.is_shutdown():
        ros_time = rospy.get_rostime()
        q.data = np.sin(ros_time.to_sec())*2
        pub.publish(q)
        rate.sleep()
