from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ArucoRecognition(Node):
    def __init__(self) -> None:
        super().__init__("aruco_recognition")
        self.subscriber = self.create_subscription(Image,"/place/holder",self.callback, qos_profile=0)


    def callback(self, msg : Image):
        self.get_logger().info(f"Image: {msg.height} x {msg.width}")

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoRecognition()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
