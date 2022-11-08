from rqt_bag_plugins.image_view import ImageView
from rqt_bag import TopicMessageView
from rqt_bag.plugins.raw_view import MessageTree

from PIL import Image
from PIL.ImageQt import ImageQt

import sys

from python_qt_binding import QT_BINDING_MODULES
from python_qt_binding.QtGui import QPen, QBrush, QPixmap
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox, QVBoxLayout, QHBoxLayout, QApplication
from python_qt_binding.QtCore import QRectF, Qt

# for code completion. Comment out when running code
# from PyQt5.QtGui import QPixmap
# from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QPen, QBrush, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QGraphicsEllipseItem


class FrameView(ImageView):
    name = 'World_View'

    def __init__(self, timeline, parent, topic):
        super(FrameView, self).__init__(timeline, parent, topic)
        # TopicMessageView.__init__(self, timeline, parent, topic)
        # self._image = None
        # self._image_topic = None
        # self._image_stamp = None
        # self.quality = Image.NEAREST
        #
        # self.world_frame = None
        #
        # self.message_tree = MessageTree(parent)
        #
        # self._overlay_font_size = 14.0
        # self._overlay_indent = (4, 4)
        # self._overlay_color = (0.2, 0.2, 1.0)
        #
        # # self.make_box('1')
        #
        # self._image_view = QGraphicsView(parent)
        # self._image_view.resizeEvent = self._resizeEvent
        # self._scene = QGraphicsScene()
        # self._image_view.setScene(self._scene)
        # parent.layout().addWidget(self._image_view)
        #
        # # parent.layout().addWidget(self.message_tree)
        # TopicMessageView.__init__(self, timeline, parent, topic)
        # parent.layout().addWidget(self.box1)
        # print(type(parent))
        self.topics = timeline._get_topics()  # once I know which topics are needed, will filter list accordingly
        self.pen = QPen()
        self.brush = QBrush(Qt.green, Qt.SolidPattern)
        self.resize = None
        # print(parent.objectName())
        # self.widget_list = QApplication.allWidgets()
        # for widget in self.widget_list:
        #     print(widget.objectName())

        # lines 256 and 266 are the infos you need to get pointer to other open plots. get_context and popup_name
        # print(type(self.timeline.popups))
        # print(len(self.timeline.popups))
        # keys = list(self.timeline.popups.keys())
        # print(keys[0])
        # print(self.timeline.popups[keys[0]])
        # print(type(parent))
        # for widget in parent.findChildren(QGraphicsView):
        #     print(widget)
        # for popup in self.timeline.popups:
        #     print(popup)



    def make_box(self, name):
        self.box1 = QGroupBox(name)

        layout = QHBoxLayout()
        layout.addWidget(QPushButton("1"))
        layout.addWidget(QPushButton("2"))
        self.box1.setLayout(layout)

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]
        # for frame in msg.frames:
        #     if frame.topic == 'frame.world':
        #         self.world_frame = frame.image

        if not msg:
            self.set_image(None, topic, 'no message')
        else:
            if topic == 'pupil_world':
                self.set_image(msg.image, topic, t)  # temporary time until I implement synced time
                # self._scene.addEllipse(QRectF(50, 100, 20, 20), self.pen, self.brush)
            elif topic == 'pupil_gaze':
                # self._scene.addEllipse(QRectF(50, 100, 20, 20), self.pen, self.brush)
                try:
                    self._scene.addEllipse(QRectF(self.resize[0]*msg.norm_pos.x, self.resize[1]*(1 - msg.norm_pos.y), 20, 20), self.pen, self.brush)
                except TypeError:
                    print('Image topic has not been published yet, will not render gaze points.')

        # if t is None:
        #     self.message_cleared()
        # else:
        #     self.message_tree.set_message(msg)

    def put_image_into_scene(self):
        if self._image:
            scale_factor = min(
                float(self._image_view.size().width() - 2) / self._image.size[0],
                float(self._image_view.size().height() - 2) / self._image.size[1])
            resized_image = self._image.resize(
                (int(scale_factor * self._image.size[0]),
                 int(scale_factor * self._image.size[1])),
                self.quality)

            self.resize = resized_image.size

            gaze = None

            scene_items = self._scene.items(Qt.DescendingOrder)
            # Assuming only one ellipse active at one time
            for item in scene_items:
                if isinstance(item, QGraphicsEllipseItem):
                    gaze = item.rect()
                    break

            QtImage = ImageQt(resized_image)
            pixmap = QPixmap.fromImage(QtImage)
            self._scene.clear()
            self._scene.addPixmap(pixmap)
            if gaze is not None:
                self._scene.addEllipse(gaze, self.pen, self.brush)

    # def message_cleared(self):
    #     super(ImageView, self).message_cleared()
    #
    # def navigate_next(self):
    #     for bag, entry in self.timeline.get_entries_with_bags(self.topics, self.timeline._timeline_frame.playhead, self.timeline._timeline_frame.end_stamp):
    #         if entry.time > self.timeline._timeline_frame.playhead:
    #             self.message_viewed(bag, self.timeline.read_message(bag, entry.position))
    #             self.timeline._timeline_frame.playhead = entry.time
    #             break
    #
    # # TODO: skips entries. fix
    # def navigate_previous(self):
    #     last_entry = None
    #     last_bag = None
    #     for bag, entry in self.timeline.get_entries_with_bags(self.topics, self.timeline._timeline_frame.start_stamp,
    #                                            self.timeline._timeline_frame.playhead):
    #         if entry.time < self.timeline._timeline_frame.playhead:
    #             last_entry = entry
    #             last_bag = bag
    #
    #     if last_entry:
    #         self.message_viewed(last_bag, self.timeline.read_message(last_bag, last_entry.position))
    #         self.timeline._timeline_frame.playhead = last_entry.time
    #
    # def navigate_first(self):
    #     for bag, entry in self.timeline.get_entries_with_bags(self.topics, *self.timeline._timeline_frame.play_region):
    #         self.message_viewed(bag, self.timeline.read_message(bag, entry.position))
    #         self.timeline._timeline_frame.playhead = entry.time
    #         break
    #
    # def navigate_last(self):
    #     last_entry = None
    #     last_bag = None
    #     for bag, entry in self.timeline.get_entries_with_bags(self.topics, *self.timeline._timeline_frame.play_region):
    #         last_entry = entry
    #         last_bag = bag
    #
    #     if last_entry:
    #         self.message_viewed(last_bag, self.timeline.read_message(last_bag, last_entry.position))
    #         self.timeline._timeline_frame.playhead = last_entry.time