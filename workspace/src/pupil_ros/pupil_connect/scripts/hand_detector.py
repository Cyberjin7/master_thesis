#!/usr/bin/env python
import mediapipe as mp
import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from pupil_msgs.msg import frame
from geometry_msgs.msg import Point
from pupil_msgs.msg import hand_pos
from pupil_msgs.msg import hand
import time

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class FrameSub:

    def __init__(self):
        self.image_sub = rospy.Subscriber('pupil_world', frame, self.callback)
        self.hand_pub = rospy.Publisher('hand_position', hand_pos, queue_size=60)
        self.hand_all_pub = rospy.Publisher('hand', hand, queue_size=60)
        self.cv_bridge = CvBridge()
        self.start_time = None
        self.tic = time.perf_counter()
        self.toc = time.perf_counter()
        self.hand_pos = hand_pos()
        self.hand_all = hand()

    def callback(self, data):
        # self.tic = time.perf_counter()
        # print(self.tic - self.toc) # time between callback
        self.toc = time.perf_counter()
        if(self.toc - self.tic >= 0.04):
            cv_image = self.cv_bridge.imgmsg_to_cv2(data.image, "bgr8") # 0.0006s average time 
            self.detect_hand(cv_image)
            cv2.waitKey(1)
            self.tic = time.perf_counter()

        # self.toc = time.perf_counter()
        # print(self.toc - self.tic) # time within callback


    def detect_hand(self, frame):
        with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5) as hands:
            # self.tic = time.perf_counter()
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # image = cv2.flip(image, 1)
            image.flags.writeable = False
            results = hands.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # self.toc = time.perf_counter()
            # print(self.toc - self.tic)
                    
            if results.multi_hand_landmarks:
                for num, hand in enumerate(results.multi_hand_landmarks):
                    mp_drawing.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS,
                                              mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4),
                                              mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2),
                                              )
                    
                # for num, hand in enumerate(results.multi_hand_landmarks):
                    if(results.multi_handedness[num].classification[0].label == 'Right'):
                        # print('Left: ') #  + str(hand.landmark[0].x))
                        # self.hand_pos.x = hand.landmark[0].x
                        # self.hand_pos.y = hand.landmark[0].y

                        # self.hand_all.landmark.clear()
                        verts = []
                        for i in range(len(hand.landmark)):
                            pos = Point()
                            pos.x = hand.landmark[i].x
                            pos.y = hand.landmark[i].y
                            pos.z = hand.landmark[i].z
                            verts.append(pos)
                            # self.hand_all.landmark.append(pos)
                        self.hand_all.landmark = verts
                        # for landmark in hand.landmark:
                        #     pos.x = landmark.x
                        #     pos.y = landmark.y
                        #     pos.z = landmark.z
                        #     self.hand_all.landmark.append(pos)

                        self.hand_pos.wrist_pos.x = hand.landmark[0].x
                        self.hand_pos.wrist_pos.y = hand.landmark[0].y
                        self.hand_pos.mft_pos.x = hand.landmark[12].x
                        self.hand_pos.mft_pos.y = hand.landmark[12].y
                        self.hand_pub.publish(self.hand_pos)
                        self.hand_all_pub.publish(self.hand_all)
                    elif(results.multi_handedness[num].classification[0].label == 'Left'):
                        # print('Right: ') # + str(hand.landmark[0].x))
                        pass
                    # print(str(num) + ": " + str(hand.landmark[0].x))
                    # keypoints = []
                    # for data_point in hand.landmark:
                    #     keypoints.append({
                    #                         'X': data_point.x,
                    #                         'Y': data_point.y,
                    #                         # 'Z': data_point.z,
                    #                         # 'Visibility': data_point.visibility,
                    #                         })
                    
                    # print(keypoints[0])

        # cv2.imshow('Hand Tracking', image)


if __name__ == '__main__':
    # rospy.init_node('hand_detector', anonymous=True)
    # frame_sub = FrameSub()
    try:
        rospy.init_node('hand_detector', anonymous=False)
        frame_sub = FrameSub()
        frame_sub.start_time = rospy.Time.now()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

