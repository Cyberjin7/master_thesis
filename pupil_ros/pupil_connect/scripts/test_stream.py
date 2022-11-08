from msgpack import loads
import zmq
import cv2
import numpy as np
# from cv_bridge import CvBridge, CvBridgeError

"""
Script to quickly output data from pupil core
Lines 29-36 show all accepted topics from pupil core
Images from frames are not printed
"""

# Why use all the string versions of the socket commands?
# For non-ambiguous utf-8 encoding! Just stick to it
context = zmq.Context()
# open a req port to talk to pupil
addr = "127.0.0.1"  # remote ip or localhost
req_port = "50020"  # same as in the pupil remote gui
req = context.socket(zmq.REQ)
req.connect("tcp://{}:{}".format(addr, req_port))
# ask for the sub port
req.send_string("SUB_PORT")
sub_port = req.recv_string()

sub = context.socket(zmq.SUB)
sub.connect("tcp://{}:{}".format(addr, sub_port))


# sub.setsockopt_string(zmq.SUBSCRIBE, "pupil.")
# sub.setsockopt_string(zmq.SUBSCRIBE, 'gaze')
# sub.setsockopt_string(zmq.SUBSCRIBE, "surfaces.")
# # sub.setsockopt_string(zmq.SUBSCRIBE, 'notify.')
# # sub.setsockopt_string(zmq.SUBSCRIBE, 'logging.')
# or everything:
# sub.setsockopt_string(zmq.SUBSCRIBE, '')
sub.setsockopt_string(zmq.SUBSCRIBE,'fixations')
# sub.setsockopt_string(zmq.SUBSCRIBE, 'frame.eye.1')

# Note to self:
# Unlike all the other messages, the frames receive 3 parts from the socket.
# First is topic, second is meta info and third the raw data.

while True:
    try:
        zmq_topic = sub.recv_string()
        zmq_hello = sub.recv_multipart()
        # zmq_topic = zmq_hello[0].decode('UTF-8') # might be better to use recv_string for topic and then use recv_multipart for rest?
        zmq_msg = loads(zmq_hello[0], raw=False)  # is a dictionary
        if len(zmq_hello) is 2:
            # zmq_msg["image_data"] = zmq_hello[2]
            pass
        print("\n{}: {}".format(zmq_topic, zmq_msg))

        # topic = sub.recv_string()
        # msg = sub.recv()
        # msg = loads(msg, raw=False)
        # print("\n{}: {}".format(topic, msg))
    except KeyboardInterrupt:
        break
