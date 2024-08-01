from PyQt5.QtCore import QThread, pyqtSignal
import rclpy
from ros2.ros2_nodes import TopicSubscriber

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