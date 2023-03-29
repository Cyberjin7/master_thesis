#! /usr/bin/ev python

import rospy
from std_msgs.msg import Float64
from custom_ros_msgs.msg import CustomData
from sync_msgs.msg import MassData
from exo_msgs.msg import state
import numpy as np



if __name__ == '__main__':
    # pub = rospy.Publisher('q', Float64, queue_size=100)
    pub = rospy.Publisher('state', state, queue_size=100)
    rospy.init_node('q_publisher', anonymous=True)
    rate = rospy.Rate(200)

    q = Float64()
    q = state()

    while not rospy.is_shutdown():
        ros_time = rospy.get_rostime()
        q.q_state.q = np.sin(ros_time.to_sec())*2
        # q.data = np.sin(ros_time.to_sec())*2
        pub.publish(q)
        rate.sleep()
