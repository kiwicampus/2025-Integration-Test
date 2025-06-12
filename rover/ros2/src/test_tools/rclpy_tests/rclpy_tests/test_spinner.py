import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

import rclpy.logging


class TestSpinner:
    """! Class to run an executor in the background while some test are performed in the main thread"""

    __test__ = False

    def __init__(self):
        self.__executor = MultiThreadedExecutor()
        self.__spin_process = None

    def __del__(self):
        rclpy.logging.get_logger("spinner").info("Spinner dies")

    def add_node(self, node: Node):
        self.__executor.add_node(node)

    def remove_node(self, node: Node):
        self.__executor.remove_node(node)

    def spin_once(self, timeout_sec: float = None):
        self.__executor.spin_once(timeout_sec)

    def spin(self):
        if self.are_nodes_spinning():
            rclpy.logging.get_logger("spinner").info("Spinnier already stared")
            return
        self.__spin_process = threading.Thread(target=self.__spin_process_target)
        self.__spin_process.start()

    def __spin_process_target(self):
        rclpy.logging.get_logger("spinner").info("Spinning start")
        self.__executor.spin()
        rclpy.logging.get_logger("spinner").info("Spinning stop")

    def cancel_spin(self):
        if self.are_nodes_spinning():
            self.__executor.shutdown(0)
            self.__spin_process.join(1.0)
            self.__spin_process = None

    def are_nodes_spinning(self):
        return (
            isinstance(self.__spin_process, threading.Thread)
            and self.__spin_process.is_alive()
        )
