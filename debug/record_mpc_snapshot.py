#!/usr/bin/env python3

import os
import shutil

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

import rosbag2_py

from cat_msgs.msg import CarState
from cat_msgs.msg import ObjectiveArrayCurv


class MpcSnapshotRecorder(Node):
    def __init__(self):
        super().__init__("mpc_snapshot_recorder")

        self.declare_parameter("planner_topic", "/as/c/locator")
        self.declare_parameter("state_topic", "/as/c/state")
        self.declare_parameter("bag_path", "mpc_snapshot_1p_2s")

        self.planner_topic = self.get_parameter("planner_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.bag_path = self.get_parameter("bag_path").value

        if os.path.exists(self.bag_path):
            shutil.rmtree(self.bag_path)

        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path,
            storage_id="mcap",
        )
        converter_options = rosbag2_py.ConverterOptions("", "")

        self.writer.open(storage_options, converter_options)

        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name=self.planner_topic,
                type="cat_msgs/msg/ObjectiveArrayCurv",
                serialization_format="cdr",
            )
        )

        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=1,
                name=self.state_topic,
                type="cat_msgs/msg/CarState",
                serialization_format="cdr",
            )
        )

        self.planner_saved = False
        self.state_count = 0

        self.planner_sub = self.create_subscription(
            ObjectiveArrayCurv,
            self.planner_topic,
            self.planner_callback,
            10,
        )

        self.state_sub = self.create_subscription(
            CarState,
            self.state_topic,
            self.state_callback,
            10,
        )

        self.get_logger().info(
            f"Waiting for 1 planner on {self.planner_topic}, then 2 states on {self.state_topic}"
        )

    def planner_callback(self, msg):
        if self.planner_saved:
            return

        t = self.get_clock().now().nanoseconds
        self.writer.write(self.planner_topic, serialize_message(msg), t)

        self.planner_saved = True
        self.get_logger().info("Saved planner message. Now saving next 2 state messages.")

    def state_callback(self, msg):
        if not self.planner_saved:
            return

        if self.state_count >= 2:
            return

        t = self.get_clock().now().nanoseconds
        self.writer.write(self.state_topic, serialize_message(msg), t)

        self.state_count += 1
        self.get_logger().info(f"Saved state message {self.state_count}/2")

        if self.state_count >= 2:
            self.get_logger().info(f"Done. Bag saved to: {self.bag_path}")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MpcSnapshotRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()