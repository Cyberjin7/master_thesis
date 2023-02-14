import rosbag
import rospy
from std_msgs.msg import String

with rosbag.Bag('test.bag', 'a') as bag:
    metadata_msg = String(data='my metadata')
    bag.write('/metadata', metadata_msg, rospy.Time(bag.get_end_time()))
    bag.close()
