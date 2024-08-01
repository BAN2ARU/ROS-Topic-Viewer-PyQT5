import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

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
