import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy_tests import TestSpinner, TestSubscriber
from rosbag_player import RosBagPlayer
from usr_msgs.msg import Fails, Fail


def verify_fail(fails_msg: Fails, fail_event: str) -> bool:
    return any(fails_msg.fails, lambda fail: fail == fail_event)


def test_collition():
    rclpy.init()

    spinner = TestSpinner()
    transient_local_qos = QoSProfile(
        depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
    )
    sub_fails = TestSubscriber("/fail_detection/fail", Fails, transient_local_qos)

    spinner.add_node(sub_fails)

    player = RosBagPlayer("TODO: your bag path")
    # TODO: do not miss play args
    player.play()

    player.join()

    fails_msg = sub_fails.get_received_msg().data
    assert verify_fail(
        fails_msg, Fail.EVENT_COLLISION
    ), "Collition event should be detected"
    # TODO: Can you test several case in a single test function?
    rclpy.shutdown()
