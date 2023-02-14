#!/usr/bin/env python


import rospy
from cv_bridge import CvBridge, CvBridgeError

from pupil_msgs.msg import frames, gaze, pupil
from sensor_msgs.msg import Image

from msgpack import loads
import zmq
import cv2
import numpy as np

"""Example code showing how to display camera images from pupil core"""

class FrameSub:

    def __init__(self):
        self.image_sub = rospy.Subscriber('pupil_frame', frames, self.callback)
        self.cv_bridge = CvBridge()

    def callback(self, data):
        for frame in data.frames:
            cv_image = self.cv_bridge.imgmsg_to_cv2(frame.image, "bgr8")
            cv2.imshow(frame.topic, cv_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    frame_subber = FrameSub()
    rospy.init_node('frame_listener', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
