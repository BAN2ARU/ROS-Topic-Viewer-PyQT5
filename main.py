import sys
from PyQt5.QtWidgets import QApplication
from ros2.ros2_manager import ROS2Manager

from ui.main_window import MainWindow

if __name__ == '__main__':
    ros2_manager = ROS2Manager()
    app = QApplication(sys.argv)
    main_window = MainWindow(ros2_manager)
    sys.exit(app.exec_())
    ros2_manager.shutdown()
