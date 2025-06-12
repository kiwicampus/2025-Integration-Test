import rclpy
from rclpy_tests import TestSpinner, TestServiceClient, TestServiceServer
from std_srvs.srv import SetBool
import time


def test_service():
    rclpy.init()

    spinner = TestSpinner()

    ping = TestServiceClient("/service", SetBool)
    pong = TestServiceServer("/service", SetBool)

    spinner.add_node(ping)
    spinner.add_node(pong)
    spinner.spin()

    try:
        assert pong.get_last_request() is None, "Data should not had been received"

        pong.set_response(SetBool.Response(success=True))
        ping.call(SetBool.Request(data=True))

        time.sleep(0.3)
        assert pong.get_last_request() is not None, "Data should had been received"
        assert pong.get_last_request().data, "Bad message value"

        assert ping.get_response().success, "Bad response value"
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    test_service()
