import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QPushButton, QTextEdit, QMainWindow, QLabel, QSizePolicy
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import re
import struct
import open3d as o3d
import math
import sensor_msgs_py.point_cloud2 as pc2
from PyQt5.QtWidgets import QOpenGLWidget
import OpenGL.GL as gl
import OpenGL.GLU as glu
import ctypes
import cv2

class ROS2Manager:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        self.executor = MultiThreadedExecutor()

    def create_node(self, node_name):
        if not rclpy.ok():
            rclpy.init(args=None)
        node = Node(node_name)
        self.executor.add_node(node)
        return node

    def spin(self):
        print("ROS2Manager: Starting spin")
        self.executor.spin()

    def shutdown(self):
        if rclpy.ok():
            print("ROS2Manager: Shutting down")
            rclpy.shutdown()

class TopicListNode(Node):
    def __init__(self):
        super().__init__('topic_list_node')

    def get_all_topics_and_types(self):
        return self.get_topic_names_and_types()

class TopicSubscriber(Node, QObject):
    msg = pyqtSignal(str, object)

    def __init__(self, topic_name, message_type, node_id):
        Node.__init__(self, f'topic_subscriber_{node_id}')
        QObject.__init__(self)
        self.topic_name = topic_name
        self.bridge = CvBridge()
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            message_type,
            topic_name,
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        if isinstance(msg, Image):
            if msg.encoding == '16UC1' :
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
                cv_image = cv2.convertScaleAbs(cv_image)  
                cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)
            else :
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.msg.emit(self.topic_name, (msg, cv_image))
        elif isinstance(msg, PointCloud2):
            cloud = self.convert_point_cloud2_to_numpy(msg)
            self.msg.emit(self.topic_name, (msg, cloud))
        else:
            self.msg.emit(self.topic_name, msg)

    def convert_point_cloud2_to_numpy(self, cloud_msg):
        points = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z, rgb = point
            rgb_uint32 = struct.unpack('I', struct.pack('f', rgb))[0]
            r = (rgb_uint32 >> 16) & 0x0000ff
            g = (rgb_uint32 >> 8) & 0x0000ff
            b = rgb_uint32 & 0x0000ff
            points.append([x, y, z, r / 255.0, g / 255.0, b / 255.0])
        return np.array(points)


    def read_points(self, cloud_msg, field_names=None, skip_nans=True):
        fmt = 'fff'  # x, y, z
        unpacker = struct.Struct(fmt).unpack_from
        for i in range(cloud_msg.height):
            for j in range(cloud_msg.width):
                point = unpacker(cloud_msg.data, (i * cloud_msg.row_step + j * cloud_msg.point_step))
                if skip_nans and any(math.isnan(c) for c in point):
                    continue
                yield point

class ROS2Thread(QThread):
    message_received = pyqtSignal(str, object)

    def __init__(self, topics, ros2_manager):
        super().__init__()
        self.topics = topics
        self.ros2_manager = ros2_manager
        self.nodes = []

    def run(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        for idx, (topic, message_type) in enumerate(self.topics):
            node = TopicSubscriber(topic, message_type, idx)
            node.msg.connect(self.handle_message)
            self.nodes.append(node)
        while rclpy.ok():
            for node in self.nodes:
                rclpy.spin_once(node, timeout_sec=0.1)

    def handle_message(self, topic, msg):
        self.message_received.emit(topic, msg)
    
    def stop(self):
        print("Stopping ROS2 thread")
        for node in self.nodes:
            node.destroy_node()
        self.ros2_manager.shutdown()
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self, ros2_manager):
        super().__init__()
        self.ros2_manager = ros2_manager
        self.initUI()

    def initUI(self):
        central_widget = QWidget()
        layout = QVBoxLayout()

        self.button = QPushButton('Topic Monitoring', self)
        self.button.clicked.connect(self.show_topic_selector)
        layout.addWidget(self.button)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        self.setWindowTitle('Main Window')

        self.resize(1200, 800)

        self.center()

        self.show()

    def center(self):
        frame_gm = self.frameGeometry()
        screen = QApplication.desktop().screenNumber(QApplication.desktop().cursor().pos())
        center_point = QApplication.desktop().screenGeometry(screen).center()
        frame_gm.moveCenter(center_point)
        self.move(frame_gm.topLeft())

    def show_topic_selector(self):
        self.topic_selector = TopicSelector(self.ros2_manager)
        self.topic_selector.move(self.geometry().center() - self.topic_selector.rect().center())
        self.topic_selector.show()

    def closeEvent(self, event):
        self.ros2_manager.shutdown()
        QApplication.instance().quit()

class TopicSelector(QWidget):
    def __init__(self, ros2_manager, parent=None):
        super().__init__(parent)
        self.ros2_manager = ros2_manager
        self.parent_window = parent
        self.initUI()

    def initUI(self):
        self.layout = QVBoxLayout()

        self.button_layout = QHBoxLayout()

        self.button = QPushButton('Show Selected Topics', self)
        self.button.clicked.connect(self.show_selected_topics)
        self.button_layout.addWidget(self.button)

        self.reset_button = QPushButton('Reset', self) 
        self.reset_button.clicked.connect(self.reset_topics)
        self.button_layout.addWidget(self.reset_button)

        self.layout.addLayout(self.button_layout)

        self.layout.addStretch()
        
        self.checkboxes_layout = QVBoxLayout()

        self.topics_and_types = self.get_ros2_topics_and_types()

        self.checkboxes = []
        self.create_topic_checkboxes()

        self.setLayout(self.layout)
        self.setWindowTitle('ROS2 Topic Selector')
        self.show()

    def create_topic_checkboxes(self):
        for topic, _ in self.topics_and_types:
            checkbox = QCheckBox(topic)
            self.checkboxes.append(checkbox)
            self.layout.addWidget(checkbox)

    def get_ros2_topics_and_types(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        node = TopicListNode()
        self.ros2_manager.executor.add_node(node)
        topics_and_types = node.get_all_topics_and_types()
        self.ros2_manager.executor.remove_node(node)
        node.destroy_node()
        return topics_and_types

    def show_selected_topics(self):
        selected_topics = [(cb.text(), self.get_message_type(cb.text())) for cb in self.checkboxes if cb.isChecked()]
        if selected_topics:
            self.echo_window = EchoWindow(selected_topics, self, self.ros2_manager)
            self.echo_window.setGeometry(self.geometry())
            self.echo_window.show()
            self.hide()

    def get_message_type(self, topic_name):
        for topic, types in self.topics_and_types:
            if topic == topic_name:
                print(f"Found message type {types[0]} for topic {topic_name}")
                return self.get_msg_type(types[0])
        print(f"No message type found for topic {topic_name}")
        return None

    def get_msg_type(self, msg_type_str):
        package_name, msg_name = msg_type_str.rsplit('/', 1)
        package_name = package_name.replace('/', '.')
        module = __import__(f"{package_name}", fromlist=[msg_name])
        return getattr(module, msg_name)
    
    def closeEvent(self, event):
        if self.parent_window:
            self.parent_window.show()
        event.accept()

    def reset_topics(self):  
        self.topics_and_types = self.get_ros2_topics_and_types()
        for checkbox in self.checkboxes:
            self.layout.removeWidget(checkbox)
            checkbox.deleteLater()
        self.checkboxes.clear()
        self.create_topic_checkboxes()

class PointCloudWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super(PointCloudWidget, self).__init__(parent)
        self.point_cloud = np.array([], dtype=np.float32)
        self.initialized = False
        self.setMinimumSize(400, 400)
        self.vbo = None
        self.point_count = 0
        print("PointCloudWidget initialized")

    def initializeGL(self):
        gl.glClearColor(0, 0, 0, 1)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glPointSize(2.0)
        self.vbo = gl.glGenBuffers(1)
        self.initialized = True
        print("OpenGL Initialized")

    def resizeGL(self, w, h):
        gl.glViewport(0, 0, w, h)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        glu.gluPerspective(45, w / h, 0.1, 50.0)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        print(f"OpenGL Viewport Resized: {w} x {h}")

    def paintGL(self):
        if not self.initialized:
            self.initializeGL()
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()
        glu.gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0)

        if self.point_count > 0:
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)
            gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
            gl.glEnableClientState(gl.GL_COLOR_ARRAY)
            gl.glVertexPointer(3, gl.GL_FLOAT, 6 * self.point_cloud.itemsize, ctypes.c_void_p(0))
            gl.glColorPointer(3, gl.GL_FLOAT, 6 * self.point_cloud.itemsize, ctypes.c_void_p(3 * self.point_cloud.itemsize))
            gl.glDrawArrays(gl.GL_POINTS, 0, self.point_count)
            gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
            gl.glDisableClientState(gl.GL_COLOR_ARRAY)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def update_point_cloud(self, point_cloud):
        sampled_point_cloud = point_cloud[::20]  
        self.point_cloud = np.array(sampled_point_cloud, dtype=np.float32)
        self.point_count = len(self.point_cloud)

        if self.point_count > 0:
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)
            gl.glBufferData(gl.GL_ARRAY_BUFFER, self.point_cloud.nbytes, self.point_cloud, gl.GL_DYNAMIC_DRAW)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

        self.update()


class EchoWindow(QMainWindow):
    def __init__(self, topics, parent, ros2_manager):
        super().__init__(parent)
        self.topics = topics
        self.parent_window = parent
        self.ros2_manager = ros2_manager
        self.initUI()
        self.setGeometry(parent.geometry())

    def initUI(self):
        self.central_widget = QWidget(self)
        self.layout = QHBoxLayout(self.central_widget)

        self.text_edits = {}
        self.image_labels = {}
        self.point_cloud_widgets = {}

        for topic, msg_type in self.topics:
            topic_layout = QVBoxLayout()
            label = QLabel(f'Echo for {topic}:')

            text_edit = QTextEdit()
            text_edit.setReadOnly(True)
            text_edit.setLineWrapMode(QTextEdit.WidgetWidth)
            text_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.text_edits[topic] = text_edit

            topic_layout.addWidget(label)
            topic_layout.addWidget(text_edit)

            if msg_type == Image:
                image_label = QLabel()
                image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                self.image_labels[topic] = image_label
                topic_layout.addWidget(image_label)
            elif msg_type == PointCloud2:
                point_cloud_widget = PointCloudWidget(self)
                self.point_cloud_widgets[topic] = point_cloud_widget
                topic_layout.addWidget(point_cloud_widget)
            self.layout.addLayout(topic_layout)

        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)
        self.setWindowTitle('Topic Echo')

        self.ros_thread = ROS2Thread(self.topics, self.ros2_manager)
        self.ros_thread.message_received.connect(self.update_text_edit)
        self.ros_thread.start()

    def update_text_edit(self, topic, data):
        if isinstance(data, tuple) and isinstance(data[0], Image):
            # Image message handling
            msg = data[0]
            cv_image = data[1]

            meta_data_str = ""
            for field_name in dir(msg):
                if field_name != 'data' and field_name != 'fields' and field_name != 'SLOT_TYPES' and field_name != 'get_fields_and_field_types' and not field_name.startswith('_'):
                    field_value = getattr(msg, field_name)
                    meta_data_str += f"{field_name}: {field_value}\n"

            def replace_commas(match):
                content = match.group(0)
                return content.replace(',', ';')

            meta_data_str = re.sub(r'\(.*?\)|\[.*?\]', replace_commas, meta_data_str, flags=re.DOTALL)

            formatted_data = '\n\n'.join(meta_data_str.split(','))

            formatted_data = formatted_data.replace(';', ',')

            if topic in self.text_edits:
                self.text_edits[topic].setPlainText(f"=== {topic} ===\n\n{formatted_data}\n")

            if topic in self.image_labels:
                h, w, ch = cv_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_BGR888)
                pixmap = QPixmap.fromImage(qt_image)
                self.image_labels[topic].setPixmap(pixmap)
                self.image_labels[topic].setScaledContents(True)
        
        elif isinstance(data, tuple) and isinstance(data[0], PointCloud2):
            # PointCloud2 message handling
            msg = data[0]
            cloud = data[1]

            meta_data_str = ""
            for field_name in dir(msg):
                if field_name != 'data' and field_name != 'fields' and field_name != 'SLOT_TYPES' and field_name != 'get_fields_and_field_types' and not field_name.startswith('_'):
                    field_value = getattr(msg, field_name)
                    meta_data_str += f"{field_name}: {field_value}\n"

            def replace_commas(match):
                content = match.group(0)
                return content.replace(',', ';')

            meta_data_str = re.sub(r'\(.*?\)|\[.*?\]', replace_commas, meta_data_str, flags=re.DOTALL)

            formatted_data = '\n\n'.join(meta_data_str.split(','))

            formatted_data = formatted_data.replace(';', ',')

            if topic in self.text_edits:
                self.text_edits[topic].setPlainText(f"=== {topic} ===\n\n{formatted_data}\n")

            points = self.convert_point_cloud2_to_open3d(cloud)
            if topic in self.point_cloud_widgets:
                self.point_cloud_widgets[topic].update_point_cloud(points)
        else:
            # Non-image message handling
            data_str = str(data)

            def replace_commas(match):
                content = match.group(0)
                return content.replace(',', ';')

            data_str = re.sub(r'\(.*?\)|\[.*?\]', replace_commas, data_str, flags=re.DOTALL)

            formatted_data = '\n\n'.join(data_str.split(','))

            formatted_data = formatted_data.replace(';', ',')

            formatted_text = f"=== {topic} ===\n\n{formatted_data}\n"
            if topic in self.text_edits:
                self.text_edits[topic].setPlainText(formatted_text)

    def convert_point_cloud2_to_open3d(self, cloud_points):
        points = []
        for point in cloud_points:
            x, y, z, r, g, b = point
            points.append([x, y, z, r, g, b])
        return np.array(points)
    
    def display_point_cloud(self, topic, cloud):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=topic)
        vis.add_geometry(cloud)
        vis.run()
        vis.destroy_window()

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros2_manager.shutdown()
        if self.parent_window:
            self.parent_window.show()
        event.accept()


if __name__ == '__main__':
    ros2_manager = ROS2Manager()
    app = QApplication(sys.argv)
    main_window = MainWindow(ros2_manager)
    sys.exit(app.exec_())
    ros2_manager.shutdown()
