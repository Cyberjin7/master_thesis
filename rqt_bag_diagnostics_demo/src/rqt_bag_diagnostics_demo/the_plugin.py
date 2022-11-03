from rqt_bag import TopicMessageView, TimelineRenderer
from rqt_bag.plugins.plugin import Plugin
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter, QBrush, QPen
from diagnostic_msgs.msg import DiagnosticStatus
import rospy
import rosbag
from rqt_bag_plugins.image_view import ImageView

# just for code completion while coding. Make sure to comment when executing
# from PyQt5.QtCore import Qt
# from PyQt5.QtWidgets import QWidget
# from PyQt5.QtGui import QPainter, QBrush, QPen


def get_color(diagnostic):
    if diagnostic.level == DiagnosticStatus.OK:
        return Qt.green
    elif diagnostic.level == DiagnosticStatus.WARN:
        return Qt.yellow
    else:
        return Qt.red


class DiagnosticTimeline(TimelineRenderer):
    def __init__(self, timeline, height=80):
        TimelineRenderer.__init__(self, timeline, msg_combine_px=height)

    def draw_timeline_segment(self, painter, topic, stamp_start, stamp_end, x, y, width, height):
        painter.setBrush(QBrush(Qt.blue))
        painter.drawRect(x, y, width, height)
        bag_timeline = self.timeline.scene()
        for bag, entry in bag_timeline.get_entries_with_bags([topic], rospy.Time(stamp_start), rospy.Time(stamp_end)):
            topic, msg, t = bag_timeline.read_message(bag, entry.position)
            color = get_color(msg)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color, 5))

            p_x = self.timeline.map_stamp_to_x(t.to_sec())
            painter.drawLine(p_x, y, p_x, y+height)


class DiagnosticPanel(TopicMessageView):
    name = 'Awesome Diagnostic'

    def __init__(self, timeline, parent, topic):
        super(DiagnosticPanel, self).__init__(timeline, parent, topic)
        self.widget = QWidget()
        parent.layout().addWidget(self.widget)
        self.msg = None
        # self.widget.paintEvent = self.paintEvent

    def message_viewed(self, bag, msg_details):
        super(DiagnosticPanel, self).message_viewed(bag, msg_details)
        # _, self.msg, _ = msg_details
        topic, msg, t = msg_details
        print("Topic is: ", topic)
        print("Time is: ", t)
        print("Message is: ", msg)
        # self.widget.update()
        bag, entry = self.timeline.get_entry(t, 'seq2')
        topic2, msg2, t2 = self.timeline.read_message(bag, entry.position)
        print(msg2)


    def paintEvent(self, event):
        self.qp = QPainter()
        self.qp.begin(self.widget)

        rect = event.rect()

        if self.msg is None:
            self.qp.fillRect(0, 0, rect.width(), rect.height(), Qt.white)
        else:
            color = get_color(self.msg)
            self.qp.setBrush(QBrush(color))
            self.qp.drawEllipse(0, 0, rect.width(), rect.height())


class DiagnosticBagPlugin(Plugin):
    def __int__(self):
        pass

    def get_view_class(self):
        return DiagnosticPanel

    def get_renderer_class(self):
        return None
        # return DiagnosticTimeline

    def get_message_types(self):
        return ['diagnostic_msgs/DiagnosticStatus', 'pupil_msgs/intbag']
