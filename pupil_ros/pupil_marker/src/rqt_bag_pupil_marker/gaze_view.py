from rqt_bag_plugins.image_view import ImageView
from rqt_bag.plugins.raw_view import MessageTree
from rqt_bag import TopicMessageView

# from PIL import Image
# from PIL.ImageQt import ImageQt

from python_qt_binding.QtGui import QPen, QBrush, QPixmap
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox, QVBoxLayout, \
    QHBoxLayout, QApplication, QGraphicsPixmapItem, QComboBox, QLabel, QTableWidget, QTableWidgetItem, QFileDialog, \
    QLineEdit
from python_qt_binding.QtCore import QRectF, Qt

# for code completion. Comment out when running code
# from PyQt5.QtGui import QPixmap
# from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QPen, QBrush, QPixmap

from PyQt5.QtWidgets import QApplication, QGraphicsPixmapItem, QComboBox, QLabel, QTableWidget, QTableWidgetItem, \
    QFileDialog, QLineEdit

import numpy as np
import pandas as pd

from os.path import exists

class GazeView(ImageView):
    name = 'Gaze'

    def __init__(self, timeline, parent, topic):
        # super(GazeView, self).__init__(timeline, parent, topic)
        TopicMessageView.__init__(self, timeline, parent, topic)

        self.message_tree = MessageTree(parent)
        parent.layout().addWidget(self.message_tree)

        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(['Timestamp', 'Marker'])
        self.table.cellDoubleClicked.connect(self.move2marker)
        # parent.layout().addWidget(self.table)

        self.marker_group = QGroupBox()
        self.marker_group.setLayout(QVBoxLayout())

        self.marker_group.layout().addWidget(self.table)

        self.label_Layout = QHBoxLayout()
        self.label_Layout.addWidget(QLabel("Current Marker:"))
        self.labels_box = QComboBox()
        self.labels_box.addItems(['Target A Start',
                                  'Target A End',
                                  'Goal A Start',
                                  'Goal A End',
                                  'Target B Start',
                                  'Target B End',
                                  'Goal B Start',
                                  'Goal B End',
                                  'Target C Start',
                                  'Target C End',
                                  'Goal C Start',
                                  'Goal C End',
                                  'Target D Start',
                                  'Target D End',
                                  'Goal D Start',
                                  'Goal D End',
                                  'Experiment Start',
                                  'Experiment End'])
        self.labels_box.setCurrentIndex(-1)
        self.label_Layout.addWidget(self.labels_box)
        self.marker_group.layout().addLayout(self.label_Layout)
        # parent.layout().addLayout(self.label_Layout)

        self.remove_current_button = QPushButton('Remove Current')
        self.remove_current_button.setEnabled(False)
        self.remove_current_button.clicked.connect(self.remove_marker)
        self.add_button = QPushButton('Add')
        self.add_button.clicked.connect(self.add_marker)

        self.button_Layout = QHBoxLayout()
        self.button_Layout.addWidget(self.remove_current_button)
        self.button_Layout.addWidget(self.add_button)
        self.marker_group.layout().addLayout(self.button_Layout)
        # parent.layout().addLayout(self.button_Layout)

        parent.layout().addWidget(self.marker_group)

        self.export_group = QGroupBox()
        self.export_group.setLayout(QVBoxLayout())

        self.file_path = QLineEdit()
        self.file_button = QPushButton('...')
        self.file_button.clicked.connect(self.choose_path)
        self.path_layout = QHBoxLayout()
        self.path_layout.addWidget(self.file_path)
        self.path_layout.addWidget(self.file_button)
        self.export_group.layout().addLayout(self.path_layout)
        # parent.layout().addLayout(self.path_layout)

        self.export_button = QPushButton('Export Markers')
        self.export_button.clicked.connect(self.export_markers)
        self.export_group.layout().addWidget(self.export_button)
        # parent.layout().addWidget(self.export_button)
        parent.layout().addWidget(self.export_group)

        self.pen = QPen()
        self.brush = QBrush(Qt.green, Qt.SolidPattern)

        self.popups = self.timeline.popups
        self.popup_keys = list(self.popups.keys())
        # self._image_view = self.popups[self.popup_keys[0]].findChild(QGraphicsView)
        self._image_view = self.popups['pupil_world__World_View'].findChild(QGraphicsView)
        self._scene = self._image_view.scene()

        self.gaze_size = 30

        self.markers = []  # TODO: Use numpy array instead. Or maybe stick to list for markers since string?
        self.timestamps = np.array([], dtype=np.float64)
        self.msg_timestamp = 0.0

        # print(self.popup_keys)
        # print(self._image_view)
        # print(self._scene)

    def add_marker(self):
        marker = self.labels_box.currentText()
        # print(marker)

        if marker != "":
            index_array = np.nonzero(self.timestamps >= self.msg_timestamp)
            if not index_array[0].size:
                self.timestamps = np.append(self.timestamps, self.msg_timestamp)
                self.markers.append(marker)
                self.table.insertRow(self.table.rowCount())

                target_index = self.table.rowCount()-1
            else:
                target_index = index_array[0][0]
                if self.timestamps[target_index] == self.msg_timestamp:
                    self.markers[target_index] = marker
                else:
                    self.timestamps = np.insert(self.timestamps, target_index, self.msg_timestamp)
                    self.markers.insert(target_index, marker)
                    self.table.insertRow(target_index)

            stamp_item = QTableWidgetItem(str(self.msg_timestamp))
            stamp_item.setFlags(stamp_item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(target_index, 0, stamp_item)

            marker_item = QTableWidgetItem(marker)
            marker_item.setFlags(marker_item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(target_index, 1, marker_item)

            self.remove_current_button.setEnabled(True)

        print(self.timestamps)
        print(self.markers)

    def remove_marker(self):
        index = np.nonzero(self.timestamps == self.msg_timestamp)[0]
        self.timestamps = np.delete(self.timestamps, index)
        self.markers.pop(index[0])

        self.table.removeRow(index[0])

        print(self.timestamps)
        print(self.markers)

        self.labels_box.setCurrentIndex(-1)
        self.remove_current_button.setEnabled(False)

    def move2marker(self, row, column):
        target_timestamp = self.table.item(row, 0)
        for bag, entry in self.timeline.get_entries_with_bags([self.topic],
                                                              *self.timeline._timeline_frame.play_region):
            msg_details = self.timeline.read_message(bag, entry.position)
            _, msg, t = msg_details[:3]
            # if msg.timestamp == float(target_timestamp.text()):
            #     self.message_viewed(bag, self.timeline.read_message(bag, entry.position))
            #     self.timeline._timeline_frame.playhead = entry.time
            #     break
            if t.to_sec() == float(target_timestamp.text()):
                self.message_viewed(bag, self.timeline.read_message(bag, entry.position))
                self.timeline._timeline_frame.playhead = entry.time
                break

    def choose_path(self):
        destination = QFileDialog.getExistingDirectory(options=QFileDialog.DontUseNativeDialog)
        if destination:  # if string not empty
            self.file_path.setText(destination)

    def export_markers(self):
        df = pd.DataFrame({'Timestamp': self.timestamps, 'Marker': self.markers})
        csv_destination = self.file_path.text() + '.csv'
        if exists(csv_destination):
            print('File already exists')
        else:
            df.to_csv(csv_destination)
            print('Exported!')

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]
        # self.msg_timestamp = msg.timestamp  # alternatively: self.msg_timestamp = t.to_sec()
        self.msg_timestamp = t.to_sec()

        # Currently assumes only one pixmap for image drawn
        scene_items = self._scene.items(Qt.DescendingOrder)
        for item in scene_items:
            if isinstance(item, QGraphicsPixmapItem):
                width = item.pixmap().width()
                height = item.pixmap().height()
                image = item.pixmap()
                self._scene.clear()
                self._scene.addPixmap(image)
                self._scene.addEllipse(QRectF(width*msg.norm_pos.x - self.gaze_size/2,
                                              height*(1-msg.norm_pos.y) - self.gaze_size/2,
                                              self.gaze_size,
                                              self.gaze_size),
                                       self.pen,
                                       self.brush)

        if t is None:
            TopicMessageView.message_cleared(self)
            self.message_tree.set_message(None)
        else:
            self.message_tree.set_message(msg)
            if self.msg_timestamp not in self.timestamps:
                self.labels_box.setCurrentIndex(-1)
                self.remove_current_button.setEnabled(False)
            else:
                index = np.nonzero(self.timestamps == self.msg_timestamp)[0][0]
                self.labels_box.setCurrentText(self.markers[index])
                self.remove_current_button.setEnabled(True)

