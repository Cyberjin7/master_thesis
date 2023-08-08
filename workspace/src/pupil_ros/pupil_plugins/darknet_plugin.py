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


import rospy
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from pupil_msgs.msg import hand


class Darknet_Visualizer(Plugin):
    """
    Plugin to subscribe to ROS topics and display relevant information on screen
    """

    def __init__(self, g_pool, topic_string = 'darknet_ros/bounding_boxes'):
        super().__init__(g_pool)
        self.order = 0.8

        self.pupil_display_list = []
        self._draw_circle_filled = draw_circle_filled_func_builder()

        self.topic = topic_string # topic_string an be used to store information between sessions

        self.bb_sub = None # bounding box subscriber
        self.hand_sub = None # hand landmarks subscriber
        self.hold_box_sub = None # held bounding box subscriber
        self.boxes = () # tuple of identified bounding box

        # UI toggles
        self.all_flag = False # flag if all bounding boxes should be shown
        self.fix_flag = False # flag if only fixated box should be shown
        self.init_flag = False # ROS initialization toggle

        self.all = True # Boolean whether all bounding boxes are drawn or not

        # Font variables
        self.glfont = fontstash.Context()
        self.glfont.add_font('opensans', ui.get_opensans_font_path())
        self.glfont.set_size(32)

        self.frame = self.g_pool.capture.frame_size

        # Chosen detectable objects and their corresponding mass
        self.objects = {"bottle": 0.5,
                        "cup": 0.1,
                        "wine glass": 0.1,
                        "mouse": 0.1,
                        "book": 0.2,
                        "keyboard": 0.1}
        
        self.grasp_strength = {"bottle": "Strong",
                               "cup": "Weak",
                               "wine glass": "Weak",
                               "mouse": "Weak",
                               "book": "Medium",
                               "keyboard": "Medium",
                               "cell phone": "Medium"}
        
        self.landmarks = () # tuple of landmark coordinates of hand

        self.hand_detected = False # Boolean to visualize hand 

        # Variables for hold detection
        self.hold_object = None # the object that is being held
        self.hold = False # whether hold is detected or not

    def init_ui(self):
        """
        Method to add UI elements to plugin screen
        """
        self.add_menu()
        self.menu.label = 'Darknet Object Detection'
        self.menu.append(ui.Button("Initialize ROS node", self.on_init_click))
        self.menu.append(ui.Button("(Like and) Subscribe", self.on_subscribe_click))
        self.menu.append(ui.Button("Unsubscribe :(", self.on_unsubscribe_click))
        self.menu.append(ui.Switch('all_flag', self, label='All Bounding Boxes: '))
        self.menu.append(ui.Switch('fix_flag', self, label='Fixated Bounding Boxes: '))
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
        """
        Method to gain information from other plugins of Pupil Capture. Here to get gaze information
        Gaze coordinates for last gaze point is saved into class attribute pupil_display_list
        """
        for pt in events.get("gaze", []):
            recent_frame_size = self.g_pool.capture.frame_size
            point = denormalize(pt["norm_pos"], recent_frame_size, flip_y=True)
            self.pupil_display_list.append((point, pt["confidence"] * 0.8))
        self.pupil_display_list[:-1] = []

    def draw_bounding_box(self, verts, box, rgba):
        """
        Method to draw bounding boxes
        
        Parameters
        ----------
        verts: List
            List of 2D coordinates. These coordinates are the corners of the bounding box
        box:
            Bounding Box Object from topic darknet_ros/bounding_boxes
        rgba: RGBA object from pyglui.cygl.utils
            Decide color of bounding box
        """
        draw_polyline_norm(verts, thickness=5, color=rgba) # Draws the box based on verts
        self.glfont.set_color_float((1.0,1.0,1.0,1.0))
        try:
            if self.hold and (self.hold_object.Class == box.Class):
                self.glfont.draw_text(box.xmin, box.ymin, box.Class + ": " + self.grasp_strength.get(box.Class) + " Grasp")
            else:
                # Writes text inside box: The name of the object and its corresponding mass
                self.glfont.draw_text(box.xmin, box.ymin, box.Class + ": " + "{:.2f}".format(self.objects.get(box.Class)) + "kg")
        except TypeError:
            # If it is an object that is not listed in class attribute objects, then text is not written
            self.glfont.draw_text(box.xmin, box.ymin, box.Class)
        self.glfont.set_align_string(v_align='left', h_align='top')

    def gl_display(self):
        """
        Method to display visual elements on the main screen of pupil capture software.

        """

        for box in self.boxes:
            # Calculate the coordinates of the corners of the bounding box
            top_left = [box.xmin/self.frame[0], 1 - box.ymin/self.frame[1]]
            top_right = [box.xmax/self.frame[0], 1 - box.ymin/self.frame[1]]
            bottom_left = [box.xmin/self.frame[0], 1 - box.ymax/self.frame[1]]
            bottom_right = [box.xmax/self.frame[0], 1 - box.ymax/self.frame[1]]

            verts = [top_left, top_right, bottom_right, bottom_left, top_left] # Organize corners into list for drawing

            # If all, then draw all bounding boxes, else only the gaze fixated bounding box
            if self.all:
                # If the bounding box that is being drawn is also the object that is detected as held object, then draw box in red instead of white
                if self.hold and (self.hold_object.Class == box.Class):
                    self.draw_bounding_box(verts, box, RGBA(1, 0, 0, 1.0))
                else:
                    self.draw_bounding_box(verts, box, RGBA(1, 1, 1, 1.0))
            else:
                # Find fixated box dependent on gaze data. If gaze coordinates fall within bounding box, this is the fixated box
                # If fixated object is also held object, draw in red. 
                # TODO: update to always show nearest object within a threshold based on centroids. 
                for pt, a in self.pupil_display_list:
                    if (box.xmin <= pt[0] <= box.xmax) and (box.ymin <= pt[1] <= box.ymax):
                        if self.hold and (self.hold_object.Class == box.Class):
                            self.draw_bounding_box(verts, box, RGBA(1, 0, 0, 1.0))
                        else:
                            self.draw_bounding_box(verts, box, RGBA(1, 1, 1, 1.0))
                        break

        # Draw lines and dots of detected hand
        if self.hand_detected:
            # Draw the connector lines between landmarks
            draw_polyline_norm([[self.landmarks[0].x, self.landmarks[0].y], 
                            [self.landmarks[1].x, self.landmarks[1].y],
                            [self.landmarks[2].x, self.landmarks[2].y],
                            [self.landmarks[3].x, self.landmarks[3].y],
                            [self.landmarks[4].x, self.landmarks[4].y]], thickness=5, color=RGBA(0, 1, 0, 1.0))
            draw_polyline_norm([[self.landmarks[0].x, self.landmarks[0].y], 
                            [self.landmarks[5].x, self.landmarks[5].y],
                            [self.landmarks[6].x, self.landmarks[6].y],
                            [self.landmarks[7].x, self.landmarks[7].y],
                            [self.landmarks[8].x, self.landmarks[8].y]], thickness=5, color=RGBA(0, 1, 0, 1.0))
            draw_polyline_norm([[self.landmarks[0].x, self.landmarks[0].y], 
                            [self.landmarks[9].x, self.landmarks[9].y],
                            [self.landmarks[10].x, self.landmarks[10].y],
                            [self.landmarks[11].x, self.landmarks[11].y],
                            [self.landmarks[12].x, self.landmarks[12].y]], thickness=5, color=RGBA(0, 1, 0, 1.0))
            draw_polyline_norm([[self.landmarks[0].x, self.landmarks[0].y], 
                            [self.landmarks[13].x, self.landmarks[13].y],
                            [self.landmarks[14].x, self.landmarks[14].y],
                            [self.landmarks[15].x, self.landmarks[15].y],
                            [self.landmarks[16].x, self.landmarks[16].y]], thickness=5, color=RGBA(0, 1, 0, 1.0))
            draw_polyline_norm([[self.landmarks[0].x, self.landmarks[0].y], 
                            [self.landmarks[17].x, self.landmarks[17].y],
                            [self.landmarks[18].x, self.landmarks[18].y],
                            [self.landmarks[19].x, self.landmarks[19].y],
                            [self.landmarks[20].x, self.landmarks[20].y]], thickness=5, color=RGBA(0, 1, 0, 1.0))
            draw_polyline_norm([[self.landmarks[5].x, self.landmarks[5].y], 
                            [self.landmarks[9].x, self.landmarks[9].y],
                            [self.landmarks[13].x, self.landmarks[13].y],
                            [self.landmarks[17].x, self.landmarks[17].y]], thickness=5, color=RGBA(0, 1, 0, 1.0)) 
            # Draw landmarks
            for landmark in self.landmarks:
                draw_points_norm([[landmark.x, landmark.y]], color=RGBA(1,0,0,1.0), size=10.0)



    def on_window_resize(self, window, w, h):
        self.frame = self.g_pool.capture.frame_size
        return super().on_window_resize(window, w, h)

    def get_init_dict(self):
        """
        Method to save variables across sessions. 
        """
        return {'topic_string':self.topic}
    
    def on_click(self, pos, button, action):
        return super().on_click(pos, button, action)
    
    def callback(self, data):
        """
        Subscriber callback for topic 'darknet_ros/bounding_boxes'.
        Saves received bounding boxes from topic into class attribute boxes.
        """
        # for box in data.bounding_boxes:
        #     print("hi")
        # print(len(data.bounding_boxes))
        self.boxes = data.bounding_boxes
        # for box in data.bounding_boxes:
        #     print(box.Class)

    def hand_callback(self, data):
        """
        Subscriber callback for topic 'hand'.
        Saves landmark coordinates into class attribute landmarks and inverts y coordinate.
        If class attribute hand_detected is False, will set to True.
        """
        if not self.hand_detected:
            self.hand_detected = True
            # print('Hand detected: ', self.hand_detected)
        self.landmarks = data.landmark
        for landmark in self.landmarks:
            landmark.y = 1 - landmark.y

    def hold_callback(self, data):
        """
        Subscriber callback for topic 'holding_object'.
        """
        if not (data.Class == "none"):
            self.hold = True
        else:
            self.hold = False
        self.hold_object = data

    def on_init_click(self):
        """
        Connected method for ros initialization UI button.
        Initializes ROS for the plugin.
        """
        rospy.init_node('pupil_ros_plugin')
        self.init_flag = True

    def on_subscribe_click(self, _=None):
        """
        Connected method for subscribe UI button.
        Subscribes to relevant ros topics.
        """
        self.bb_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        self.hand_sub = rospy.Subscriber('hand', hand, self.hand_callback)
        self.hold_box_sub = rospy.Subscriber('holding_object', BoundingBox, self.hold_callback)
    
    def on_unsubscribe_click(self, _=None):
        """
        Connected method for unsubscribe UI button.
        Unsubscribes to all topics and clears related variables.
        """
        self.bb_sub.unregister()
        self.hand_sub.unregister()
        self.hold_box_sub.unregister()
        self.boxes.clear()
        self.landmarks.clear()

    def on_change_click(self):
        """
        Connected method for change UI button.
        By toggling the flag buttons in the UI, the class attribute 'all' can be changed.
        'all' is used to decide whether all bounding boxes are visualized or only the target box is shown.
        Fail safe: both flags cannot be true or false at the same time. 
        """
        if(self.all_flag and not self.fix_flag):
            self.all = True
            print('Change to all')
        elif(not self.all_flag and self.fix_flag):
            self.all = False
            print('Change to 1 only')
        elif(self.all_flag and self.fix_flag):
            print('Both no work')
        else:
            print('No option selected')
