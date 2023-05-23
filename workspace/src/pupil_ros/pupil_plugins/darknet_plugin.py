"""
(*)~---------------------------------------------------------------------------
Pupil - eye tracking platform
Copyright (C) Pupil Labs

Distributed under the terms of the GNU
Lesser General Public License (LGPL v3.0).
See COPYING and COPYING.LESSER for license details.
---------------------------------------------------------------------------~(*)
"""

from gl_utils import draw_circle_filled_func_builder
from methods import denormalize
from plugin import Plugin
from pyglui.cygl.utils import RGBA, draw_points_norm, draw_points, draw_polyline_norm, draw_rounded_rect, draw_x
from pyglui.pyfontstash import fontstash
import numpy as np
import cv2
from pyglui import ui

# logging
import logging
logger = logging.getLogger(__name__)

import rospy
from darknet_ros_msgs.msg import BoundingBoxes


class Darknet_Visualizer(Plugin):
    """
    DisplayGaze shows the three most
    recent gaze position on the screen
    """

    def __init__(self, g_pool, topic_string = 'darknet_ros/bounding_boxes'):
        super().__init__(g_pool)
        self.order = 0.8

        self.pupil_display_list = []
        self._draw_circle_filled = draw_circle_filled_func_builder()

        self.topic = topic_string
        self.bb_sub = None
        self.fix_box_sub = None
        self.boxes = ()
        self.all_flag = False
        self.fix_flag = False
        self.init_flag = False

        self.all = True

        self.glfont = fontstash.Context()
        self.glfont.add_font('opensans', ui.get_opensans_font_path())
        self.glfont.set_size(32)

        self.frame = self.g_pool.capture.frame_size

        self.objects = {"bottle": 1.0,
                        "cup": 0.1,
                        "wine glass": 0.1,
                        "mouse": 0.1,
                        "book": 0.2}

    def init_ui(self):
        self.add_menu()
        self.menu.label = 'Darknet Object Detection'
        self.menu.append(ui.Button("Initialize ROS node", self.on_init_click))
        self.menu.append(ui.Button("(Like and) Subscribe", self.on_subscribe_click))
        self.menu.append(ui.Button("Unsubscribe :(", self.on_unsubscribe_click))
        self.menu.append(ui.Switch('all_flag', self, label='All Bounding Boxes: '))
        self.menu.append(ui.Switch('fix_flag', self, label='Fixated Bounding Boxes: '))
        # self.menu.append(ui.Text_Input("topic", self, label="Topic: "))
        self.menu.append(ui.Button("Change", self.on_change_click))


    def deinit_ui(self):
        self.remove_menu()
        return super().deinit_ui()
    
    def cleanup(self):
        if(self.all_flag or self.fix_flag):
            self.bb_sub.unregister()
        if(self.init_flag):
            rospy.signal_shutdown('Deactivating Plugin')
        return super().cleanup()

    def recent_events(self, events):
        for pt in events.get("gaze", []):
            recent_frame_size = self.g_pool.capture.frame_size
            point = denormalize(pt["norm_pos"], recent_frame_size, flip_y=True)
            self.pupil_display_list.append((point, pt["confidence"] * 0.8))
        self.pupil_display_list[:-1] = []

    def draw_bounding_box(self, verts, box):
        draw_polyline_norm(verts, thickness=5, color=RGBA(1, 1, 1, 1.0))   
        self.glfont.set_color_float((1.0,1.0,1.0,1.0))
        try:
            self.glfont.draw_text(box.xmin, box.ymin, box.Class + ": " + "{:.2f}".format(self.objects.get(box.Class)) + "kg")
        except TypeError:
            self.glfont.draw_text(box.xmin, box.ymin, box.Class)
        # self.glfont.draw_text(box.xmin, box.ymin, box.Class + ": " +"{:.1f}".format(box.probability*100))
        self.glfont.set_align_string(v_align='left', h_align='top')

    def gl_display(self):
        # for pt, a in self.pupil_display_list:
        #     # This could be faster if there would be a method to also add multiple colors per point
        #     self._draw_circle_filled(
        #         tuple(pt),
        #         size=35 / 2,
        #         color=RGBA(1.0, 0.2, 0.4, a),
        #     )
        #     print(pt)

        for box in self.boxes:
            top_left = [box.xmin/self.frame[0], 1 - box.ymin/self.frame[1]]
            top_right = [box.xmax/self.frame[0], 1 - box.ymin/self.frame[1]]
            bottom_left = [box.xmin/self.frame[0], 1 - box.ymax/self.frame[1]]
            bottom_right = [box.xmax/self.frame[0], 1 - box.ymax/self.frame[1]]

            verts = [top_left, top_right, bottom_right, bottom_left, top_left]

            if self.all:
                self.draw_bounding_box(verts, box)
            else:
                for pt, a in self.pupil_display_list:
                    if (box.xmin <= pt[0] <= box.xmax) and (box.ymin <= pt[1] <= box.ymax):

                        # draw_polyline_norm(verts, thickness=5, color=RGBA(1, 1, 1, 1.0))   
                        # self.glfont.set_color_float((1.0,1.0,1.0,1.0))
                        # try:
                        #     self.glfont.draw_text(box.xmin, box.ymin, box.Class + ": " + "{:.2f}".format(self.objects.get(box.Class)) + "kg")
                        # except TypeError:
                        #     self.glfont.draw_text(box.xmin, box.ymin, box.Class)
                        # # self.glfont.draw_text(box.xmin, box.ymin, box.Class + ": " +"{:.1f}".format(box.probability*100))
                        # self.glfont.set_align_string(v_align='left', h_align='top')
                        self.draw_bounding_box(verts, box)
                        break

        # self.boxes = []


    def on_window_resize(self, window, w, h):
        self.frame = self.g_pool.capture.frame_size
        return super().on_window_resize(window, w, h)

    def get_init_dict(self):
        return {'topic_string':self.topic}
    
    def on_click(self, pos, button, action):
        print(button)
        print(action)
        return super().on_click(pos, button, action)
    
    def callback(self, data):
        # for box in data.bounding_boxes:
        #     print("hi")
        # print(len(data.bounding_boxes))
        self.boxes = data.bounding_boxes
        # for box in data.bounding_boxes:
        #     print(box.Class)

    def on_init_click(self):
        rospy.init_node('pupil_ros_plugin')
        self.init_flag = True

    def on_subscribe_click(self, _=None):
        self.bb_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        # if(self.all_flag and not self.fix_flag):
        #     if(self.bb_sub is not None):
        #         self.bb_sub.unregister()
        #     self.bb_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        # elif(not self.all_flag and self.fix_flag):
        #     if(self.bb_sub is not None):
        #         self.bb_sub.unregister()
        #     self.bb_sub = rospy.Subscriber('fixated_objects', BoundingBoxes, self.callback)
        # elif(self.all_flag and self.fix_flag):
        #     logger('Cannot subscribe to both at once')
        # else:
        #     logger('Topic not selected')
    
    def on_unsubscribe_click(self, _=None):
        self.bb_sub.unregister()
        self.boxes.clear()

    def on_change_click(self):
        if(self.all_flag and not self.fix_flag):
            self.all = True
        elif(not self.all_flag and self.fix_flag):
            self.all = False
        elif(self.all_flag and self.fix_flag):
            logger('Both no work')
        else:
            logger('No option selected')
