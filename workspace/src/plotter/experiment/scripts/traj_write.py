import rosbag
import rospy
from std_msgs.msg import Int32, String, Float64
import numpy as np

# bag = rosbag.Bag('test2.bag', 'w')

# try:
#     # s = String()
#     # s.data = 'foo'
#
#     i1 = intbag()
#     i2 = intbag()
#     i2.data = 5
#     # i.data = 42
#
#     # bag.write('chatter', s)
#     # bag.write('numbers', i)
#     for counter in range(100):
#         now = rospy.Time.now()
#         i1.data = counter
#         i1.header.stamp = now
#         i1.header.seq = counter
#         i2.header.stamp = now
#         i2.header.seq = counter
#         bag.write('seq1', i1)
#         bag.write('seq2', i2)
#
# finally:
#     bag.close()


def bag_writer():
    rospy.init_node('bag_writer', anonymous=True)
    rate = rospy.Rate(200)

    bag = rosbag.Bag('trajectory.bag', 'w')

    traj = Float64()

    start_time = rospy.get_rostime()

    while not rospy.is_shutdown():
        ros_time = rospy.get_rostime() - start_time
        if (ros_time <= rospy.Duration(30)):
            traj.data = 55 - 45*np.cos(ros_time.to_sec())
            bag.write('traj', traj)
        else:
            break
        rate.sleep()

    bag.close()
    print("Done writing!")


if __name__ == '__main__':
    try:
        bag_writer()
    except rospy.ROSInterruptException:
        pass
