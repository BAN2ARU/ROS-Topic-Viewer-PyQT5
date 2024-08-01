from PyQt5.QtWidgets import QOpenGLWidget, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QLabel, QSizePolicy
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
import OpenGL.GL as gl
import OpenGL.GLU as glu
import numpy as np
import ctypes
import re
from sensor_msgs.msg import Image, PointCloud2
from ros2.ros2_thread import ROS2Thread
import open3d as o3d

def replace_commas(match):
    content = match.group(0)
    return content.replace(',', ';')


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