from rosbag_player import RosBagPlayer
import time
import os.path


import rclpy
from rclpy.node import Node


def test_rosbag_player():
    try:
        player = RosBagPlayer("/workspace/rosbags/does_not_exist.mcap")
        assert False, "This should rise and exception"
    except Exception as e:
        print(str(e), flush=True)

    player = RosBagPlayer("/workspace/rosbags/test_bag.mcap")

    player.play()

    time.sleep(1)
    rclpy.init()
    node = Node("dummy")
    nodes_list = node.get_node_names()
    assert len(nodes_list) > 1, "no rosbag player launched"

    player.join()
    rclpy.shutdown()
