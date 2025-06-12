import rclpy
from rclpy_tests import TestSpinner, TestSubscriber, TestPublisher
from std_msgs.msg import Bool
import time


def test_topic():

    rclpy.init()

    spinner = TestSpinner()

    ping = TestPublisher("/topic", Bool)
    pong = TestSubscriber("/topic", Bool)

    spinner.add_node(ping)
    spinner.add_node(pong)

    try:
        spinner.spin()
        assert not pong.has_data_been_received(), "Data should not had been received"

        # in loop publisher
        pub_period = 0.2
        ping.publish(msg=Bool(data=True), period=pub_period, times=10)
        time.sleep(3.0)
        assert pong.has_data_been_received(), "Data should had been received"
        assert pong.get_received_msg().data, "Bad message value"
        assert pong.msg_count == 10, "Bad topic messages count"

        # single publisher
        ping.publish(msg=Bool(data=True))
        time.sleep(0.1)
        assert pong.msg_count == 11, "Bad topic messages count"

        # test cancel spin
        spinner.cancel_spin()
        assert not spinner.are_nodes_spinning(), "error stopping spinner"

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    test_topic()
