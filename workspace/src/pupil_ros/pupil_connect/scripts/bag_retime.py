import rosbag

with rosbag.Bag('bags/test_retimed.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('bags/test2.bag').read_messages():
        outbag.write(topic, msg, msg.header.stamp)
