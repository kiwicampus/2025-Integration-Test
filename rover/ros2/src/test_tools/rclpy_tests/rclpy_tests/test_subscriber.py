from typing import List
from itertools import pairwise

from rclpy.node import Node
from rclpy.time import Time


class TestSubscriber(Node):
    """! Simple Subscriber node, for testing purposes"""

    __test__ = False

    def __init__(self, topic: str, msg_type: any, qos_profile=1):
        """!
        @param topic (str) Topic name
        @param type (any) Topic message type
        @param qos_profile(QosProfile | int) qos_profile object or qos deep
        """
        node_name = "subscriber_" + topic.replace("/", "_")
        super().__init__(node_name)

        self.subscription = self.create_subscription(
            msg_type, topic, self.callback, qos_profile=qos_profile
        )

        self.__msg = None
        self.msg_count = 0
        self.__time: List[Time] = []

    def callback(self, msg):
        """!
        Save topic message
        @param msg topic message
        """
        self.__time.append(self.get_clock().now().nanoseconds)
        self.msg_count += 1
        self.__msg = msg

    def has_data_been_received(self):
        return self.__msg is not None

    def get_received_msg(self):
        return self.__msg

    def get_frequency(self):
        """! Compute topic frequency
        @return average frequency
        """
        if len(self.__time) <= 1:
            return 0

        dt = [y - x for (x, y) in pairwise(self.__time)]

        average_period = sum(dt) * 1e-9 / len(dt)
        average_frequency = 1 / average_period

        return average_frequency
