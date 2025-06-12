from rclpy.node import Node


class TestPublisher(Node):
    """! Simple Publisher node, for testing purposes"""

    __test__ = False

    def __init__(self, topic: str, msg_type: any, qos_profile=1):
        """!
        @param topic (str) Topic name
        @param type (any) Topic message type
        @param qos_profile(QosProfile | int) qos_profile object or qos deep
        """
        node_name = "publisher_" + topic.replace("/", "_")
        super().__init__(node_name)

        self.publisher = self.create_publisher(msg_type, topic, qos_profile)
        self.__times = -1
        self.__msg = None
        self.__publish_tmr = None

    def set_published_msg(self, msg: any):
        self.__msg = msg

    def publish(self, *, msg: any = None, period: float = -1.0, times: int = -1):
        """! Publish a message

        @param msg (MsgType, optional) Message to be published, use message set if None
        @param period (float, optional) if greater than 0, publish periodically the message with the period as seconds. Defaults to -1.0.
        @param times (int, optional) Publish repetitions number. If zero or negative, publish forever. Defaults to -1.
        """
        if msg is not None:
            self.__msg = msg

        if period > 0.0:
            self.__times = times
            if self.__publish_tmr is not None:
                self.__publish_tmr.cancel()
            self.__publish_tmr = self.create_timer(period, self.timer_cb)
        else:
            self.publisher.publish(msg)

    def timer_cb(self):
        """! timer callback for periodic publisher"""
        if self.__times > 0:
            self.__times -= 1
        elif self.__times == 0:
            self.__publish_tmr.cancel()
            return

        self.publisher.publish(self.__msg)
