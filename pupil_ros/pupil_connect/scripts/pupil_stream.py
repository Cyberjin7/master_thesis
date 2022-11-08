#!/usr/bin/env python
import geometry_msgs.msg
import rospy
from cv_bridge import CvBridge, CvBridgeError

from pupil_msgs.msg import frame, gaze, pupil, surface, gaze_surface, fixation_surface, frames, fixation
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from msgpack import loads
import zmq
import cv2
import numpy as np


def data2point(data):
    point = geometry_msgs.msg.Point()
    point.x = data[0]
    point.y = data[1]
    if len(data) > 2:
        point.z = data[2]
    return point


class PupilStreamer:

    FRAME_FORMAT = "bgr"

    # TODO: Receive argument from ros console for ip and port. Make launch file + parameter file?
    def __init__(self, topics, ip="127.0.0.1", pupil_port="50020"):
        self.surface_pub = None
        self.pupil_pub = None
        self.gaze_pub = None
        self.frame_pub = None
        self.world_pub = None
        self.eyes_pub = None
        self.front_camera_pub = None
        self.fixation_pub = None
        self.context = zmq.Context()
        self.addr = ip
        self.req_port = pupil_port
        self.req = self.context.socket(zmq.REQ)
        self.req.connect("tcp://{}:{}".format(self.addr, self.req_port))
        self.req.send_string("SUB_PORT")
        self.sub_port = self.req.recv_string()
        self.sub = self.context.socket(zmq.SUB)
        self.sub.connect("tcp://{}:{}".format(self.addr, self.sub_port))
        self.topics = topics
        self.cv_bridge = CvBridge()
        self.frames_list = frames()

    def subscribe(self):
        for topic in self.topics:
            self.sub.setsockopt_string(zmq.SUBSCRIBE, topic)
            if topic == 'frame':
                # self.frame_pub = rospy.Publisher('pupil_frame', frames, queue_size=300)
                self.world_pub = rospy.Publisher('pupil_world', frame, queue_size=60)
                self.front_camera_pub = rospy.Publisher('camera_stream', Image, queue_size=60)
                self.eyes_pub = rospy.Publisher('pupil_eyes', frame, queue_size=120)
                print("Publishing world and eye frame data")
            # elif topic == 'world':
            #     self.world_pub = rospy.Publisher('pupil_world', frame, queue_size=60)
            #     print("Publishing world frame data")
            # elif topic == 'eyes':
            #     self.eyes_pub = rospy.Publisher('pupil_eyes', frame, queue_size=120)
            #     print("Publishing eye frame data")
            elif topic == 'gaze':
                self.gaze_pub = rospy.Publisher('pupil_gaze', gaze, queue_size=120)
                print("Publishing gaze data")
            elif topic == 'pupil':
                self.pupil_pub = rospy.Publisher("pupil_pupil", pupil, queue_size=120)
                print("Publishing pupil data")
            elif topic == 'surface':
                self.surface_pub = rospy.Publisher("pupil_surface", surface, queue_size=120)
                print("Publishing surface data")
            elif topic == 'fixation':
                self.fixation_pub = rospy.Publisher('pupil_fixation', fixation, queue_size=120)
                print("Publishing fixation data")

    def publish(self):
        zmq_topic = self.sub.recv_string()
        zmq_message = self.sub.recv_multipart()

        payload = loads(zmq_message[0], raw=False)
        if len(zmq_message) == 2:
            extra_frame = [zmq_message[1]]
            payload["image_data"] = extra_frame

        if zmq_topic.startswith("gaze"):
            gaze_msg = gaze()
            gaze_msg.topic = payload["topic"]
            gaze_msg.confidence = payload["confidence"]
            gaze_msg.timestamp = payload["timestamp"]
            # TODO: eye_center_3d and gaze_normal_3d key differs depending if monocular or binocular. Account for this
            # gaze_msg.eye_center_3d = data2point(payload["eye_center_3d"])  # 3 elements
            # gaze_msg.gaze_normal_3d = data2point(payload["gaze_normal_3d"])  # 3 elements
            gaze_msg.gaze_point_3d = data2point(payload["gaze_point_3d"])  # 3 elements
            gaze_msg.norm_pos = data2point(payload["norm_pos"])  # 2 elements
            self.gaze_pub.publish(gaze_msg)
        elif zmq_topic.startswith("pupil."):
            pupil_msg = pupil()
            pupil_msg.id = payload["id"]
            pupil_msg.topic = payload["topic"]
            pupil_msg.method = payload["method"]
            pupil_msg.norm_pos = data2point(payload["norm_pos"])  # 2 elements
            pupil_msg.diameter = payload["diameter"]
            pupil_msg.confidence = payload["confidence"]
            pupil_msg.timestamp = payload["timestamp"]
            pupil_msg.ellipse_center = data2point(payload["ellipse"].get("center"))  # 2 elements
            pupil_msg.ellipse_axes = data2point(payload["ellipse"].get("axes"))  # 2 elements
            pupil_msg.ellipse_angle = payload["ellipse"].get("angle")
            if pupil_msg.method == "pye3d 0.3.0 real-time":
                pupil_msg.sphere_center = data2point(payload["sphere"].get("center"))  # 3 elements
                pupil_msg.sphere_radius = payload["sphere"].get("radius")
                pupil_msg.projected_sphere_center = data2point(payload["projected_sphere"].get("center"))  # 2 elements
                pupil_msg.projected_sphere_axes = data2point(payload["projected_sphere"].get("axes"))  # 2 elements
                pupil_msg.projected_sphere_angle = payload["projected_sphere"].get("angle")
                pupil_msg.circle_3d_center = data2point(payload["circle_3d"].get("center"))  # 3 elements
                pupil_msg.circle_3d_normal = data2point(payload["circle_3d"].get("normal"))  # 3 elements
                pupil_msg.circle_3d_radius = payload["circle_3d"].get("radius")
                pupil_msg.diameter_3d = payload["diameter_3d"]
                pupil_msg.location = data2point(payload["location"])  # 2 elements
                pupil_msg.model_confidence = payload["model_confidence"]
                pupil_msg.theta = payload["theta"]
                pupil_msg.phi = payload["phi"]
            self.pupil_pub.publish(pupil_msg)
        elif zmq_topic.startswith("surfaces"):
            surface_msg = surface()
            surface_msg.topic = payload["topic"]
            surface_msg.name = payload["name"]
            # TODO: Implement rotation matrix to quaternion transformation
            # surface_msg.surf_to_img_trans =
            # surface_msg.img_to_surf_trans =
            surface_msg.timestamp = payload["timestamp"]
            for surface_gaze in payload["gaze_on_surfaces"]:
                gazes_msg = gaze_surface()
                gazes_msg.topic = surface_gaze["topic"]
                gazes_msg.norm_pos = data2point(surface_gaze["norm_pos"])
                gazes_msg.confidence = surface_gaze["confidence"]
                gazes_msg.on_surf = surface_gaze["on_surf"]
                gazes_msg.timestamp = surface_gaze["timestamp"]
                surface_msg.gazes.append(gazes_msg)
            for surface_fix in payload["fixations_on_surfaces"]:
                fix_msg = fixation_surface()
                fix_msg.topic = surface_fix["topic"]
                fix_msg.norm_pos = data2point(surface_fix["norm_pos"])
                fix_msg.confidence = surface_fix["confidence"]
                fix_msg.on_surf = surface_fix["on_surf"]
                fix_msg.timestamp = surface_fix["timestamp"]
                fix_msg.id = surface_fix["id"]
                fix_msg.duration = surface_fix["duration"]
                fix_msg.dispersion = surface_fix["dispersion"]
                surface_msg.fixations.append(fix_msg)
            self.surface_pub.publish(surface_msg)
        elif zmq_topic.startswith("frame"):
            if payload["format"] != self.FRAME_FORMAT:
                print(f"different frame format ({payload['format']});")

            frame_msg = frame()
            frame_msg.topic = payload["topic"]
            frame_msg.width = payload["width"]
            frame_msg.height = payload["height"]
            frame_msg.index = payload["index"]
            frame_msg.timestamp = payload["timestamp"]
            frame_msg.format = payload["format"]
            cv_image = np.frombuffer(payload["image_data"][0], dtype=np.uint8).reshape(payload["height"], payload["width"], 3)
            frame_msg.image = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            image_msg = frame_msg.image

            # topic_list = [x.topic for x in self.frames_list.frames]
            # if frame_msg.topic not in topic_list:
            #     self.frames_list.frames.append(frame_msg)
            # else:
            #     frame_index = topic_list.index(frame_msg.topic)
            #     self.frames_list.frames[frame_index] = frame_msg
            #
            # if len(self.frames_list.frames) == 3:
            #     self.frame_pub.publish(self.frames_list)
            #     self.frames_list.frames.clear()

            if frame_msg.topic.startswith('frame.world'):
                self.world_pub.publish(frame_msg)
                self.front_camera_pub.publish(image_msg)
            elif frame_msg.topic.startswith('frame.eye'):
                self.eyes_pub.publish(frame_msg)
        elif zmq_topic.startswith("fixation"):
            fixation_msg = fixation()
            fixation_msg.topic = payload["topic"]
            fixation_msg.id = payload["id"]
            fixation_msg.timestamp = payload["timestamp"]
            fixation_msg.duration = payload["duration"]
            fixation_msg.norm_pos[0] = payload["norm_pos"][0]
            fixation_msg.norm_pos[1] = payload["norm_pos"][1]
            fixation_msg.dispersion = payload["dispersion"]
            fixation_msg.confidence = payload["confidence"]
            fixation_msg.method = payload["method"]
            if fixation_msg.method == "3d gaze":
                fixation_msg.gaze_point_3d = data2point(payload["gaze_point_3d"])
            fixation_msg.base_data = []
            for data in payload["base_data"]:
                fixation_msg.base_data.append(data[1])
            self.fixation_pub.publish(fixation_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('pupil_stream', anonymous=True)
        rate = rospy.Rate(660)  # rospy.Rate(660)
        pupil_topics = rospy.get_param('pupil_stream/topics')
        pupil_stream = PupilStreamer(pupil_topics)
        pupil_stream.subscribe()

        while not rospy.is_shutdown():
            pupil_stream.publish()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
