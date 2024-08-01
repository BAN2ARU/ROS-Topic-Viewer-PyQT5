from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QPushButton
from ros2.ros2_nodes import TopicListNode
from ui.widgets import EchoWindow
import rclpy


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