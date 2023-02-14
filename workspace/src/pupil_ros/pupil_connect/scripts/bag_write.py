import rosbag
import rospy
from std_msgs.msg import Int32, String
from pupil_msgs.msg import intbag

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

    bag = rosbag.Bag('test3.bag', 'w')
    try:
        i1 = intbag()
        i2 = intbag()

        for counter in range(100):
            now = rospy.Time.now()
            i1.data = counter
            i1.header.stamp = now
            i1.header.seq = counter
            i2.header.stamp = now
            i2.header.seq = counter
            bag.write('seq1', i1, now)  # bag.write('seq1', i1)
            bag.write('seq2', i2, now)  # bag.write('seq'2, i2)

    finally:
        bag.close()
    print("Done writing!")


if __name__ == '__main__':
    try:
        bag_writer()
    except rospy.ROSInterruptException:
        pass
