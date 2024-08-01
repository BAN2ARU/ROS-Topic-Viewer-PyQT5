from rclpy.node import Node
from rclpy.qos import QoSProfile
from PyQt5.QtCore import QObject, pyqtSignal
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import struct
import cv2
import math

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