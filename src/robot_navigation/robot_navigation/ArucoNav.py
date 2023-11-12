#!/usr/bin/env python

import rclpy

from rclpy.node import Node
from robot_vision_msgs.msg import ArucoRecognitions

class ArucoNav(Node):
    def __init__(self) -> None:
        super().__init__("aruco_nav")
        self.recognitions_sub = self.create_subscription(ArucoRecognitions, "robot_vision/ar/recognitions", self.callback, qos_profile=1)

    
    def callback(self, msg :  ArucoRecognitions) -> None:
        self.get_logger().error(f"{msg}")

        return

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()