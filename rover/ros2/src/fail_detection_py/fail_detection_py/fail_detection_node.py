#!/usr/bin/env python3

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from usr_msgs.msg import Fails


def headers2dt(header1: Header, header2: Header):
    """Calculate time difference between two message headers in nanoseconds"""
    dt_ns = (header1.stamp.nanosec - header2.stamp.nanosec) + (
        header1.stamp.sec - header2.stamp.sec
    ) * 1e9
    return dt_ns


class FailDetector(Node):
    def __init__(self):
        super().__init__("fail_detector")

        # Environment variables
        self.imu_no_msgs_report_time = int(
            os.getenv("FAIL_DETECTION_IMU_NO_MSGS_REPORT_TIME", 5)
        )

        # Class variables
        self.bot_speed = 0.0
        self.motion_state = "forward"

        default_sub_qos = QoSProfile(depth=1)
        transient_local_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create subscribers
        self.subs_imu_camera = self.create_subscription(
            Imu, "/camera/imu", self.imu_cb, default_sub_qos
        )

        self.subs_imu_chassis = self.create_subscription(
            Imu, "/imu/data", self.chassis_imu_cb, default_sub_qos
        )

        self.subs_bot_speed = self.create_subscription(
            Odometry,
            "/wheel_odometry/local_odometry",
            self.bot_speed_cb,
            default_sub_qos,
        )

        # Create publishers
        self.pub_fail = self.create_publisher(
            Fails, "/fail_detection/fail", transient_local_qos
        )

        ## Timers
        # Add some timer if you need it
        # self.dummy_tmr = self.create_timer(0.3, self.timer_callback)
        # self.dummy_tmr.cancel()

        self.get_logger().info("Fail detector constructor: Success!")

    def imu_cb(self, msg: Imu) -> None:
        """Receive the imu msg and process it to detect collisions"""
        # TODO:
        # Detect collitions

    def chassis_imu_cb(self, msg: Imu) -> None:
        """Receive the chassis imu msg and process it to detect pitch changes"""
        # Chassis Imu is aligned with the robot base link frame
        # TODO:
        # Detect collitions

    def bot_speed_cb(self, msg: Odometry) -> None:
        """Receive the speed command of the robot to know if its going forward or backwards"""
        self.bot_speed = msg.twist.twist.linear.x
        if self.bot_speed > 0.0 and self.motion_state != "forward":
            self.motion_state = "forward"

        elif self.bot_speed < 0.0 and self.motion_state != "backwards":
            self.motion_state = "backwards"


def main(args=None):
    rclpy.init(args=args)

    fail_detection_node = FailDetector()

    node_period_s = int(os.getenv("FAIL_DETECTION_NODE_PERIOD", "50")) * 1e-3

    try:
        while rclpy.ok():
            rclpy.spin_once(fail_detection_node)
            time.sleep(node_period_s)

    except KeyboardInterrupt:
        pass

    fail_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
