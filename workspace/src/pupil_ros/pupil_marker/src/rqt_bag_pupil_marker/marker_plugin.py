from rqt_bag.plugins.plugin import Plugin
from .frame_view import FrameView
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget

import rospy
import rosbag

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget



class PupilMarkerPlugin(Plugin):
    def __init__(self):
        pass

    def get_view_class(self):
        return FrameView

    def get_renderer_class(self):
        return None

    def get_message_types(self):
        return['pupil_msgs/frame']  # only for now until I figure out all parts of the code out