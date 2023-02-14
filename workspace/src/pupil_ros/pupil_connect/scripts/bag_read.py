import rosbag

bag = rosbag.Bag('bags/test_retimed.bag')
for topic, msg, t in bag.read_messages(topics=['seq1', 'seq2']):
    print(topic)
    print(msg)
    print(t)
bag.close()
