from PyQt5.QtWidgets import QMainWindow, QPushButton, QWidget, QVBoxLayout, QApplication
from ui.select_topic import TopicSelector


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