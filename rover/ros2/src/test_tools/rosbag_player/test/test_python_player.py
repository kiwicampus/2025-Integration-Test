from rosbag_player import RosBagPlayer
import time
import os.path


import rclpy
from rclpy.node import Node


def test_rosbag_player():
    player = RosBagPlayer("gs://autonomy-vision/rosbags/test_bag/test_bag_0.mcap")
    assert os.path.isfile(player.local_path), "file was not downloaded"

    player.play()

    time.sleep(1)
    rclpy.init()
    node = Node("dummy")
    nodes_list = node.get_node_names()
    assert len(nodes_list) > 1, "no rosbag player launched"

    player.join()
    rclpy.shutdown()
