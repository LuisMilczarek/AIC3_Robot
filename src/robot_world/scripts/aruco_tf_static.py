#!/usr/bin/env python

import rclpy
import numpy as np


from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

class ArucoStaticFramePubliser(Node):
    def __init__(self) -> None:
        super().__init__("aruco_static_transform_publisher")
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()
    
    def make_transforms(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "/map"
        t.child_frame_id = "/marker_1"

        #  pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
        #     'y': LaunchConfiguration('y_pose', default='0.00'),
        #     'z': LaunchConfiguration('z_pose', default='4.00'),
        #     'R': LaunchConfiguration('roll', default='0.00'),
        #     'P': LaunchConfiguration('pitch', default='1.57079632679'),
        #     'Y': LaunchConfiguration('yaw', default='0.00')}
        
        q = quaternion_from_euler(np.pi/2, 1.57079632679, 0)

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 4.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_static_broadcaster.sendTransform(t)

        self.get_logger().info("Cheguei")

        

    

def main(args=None):
    rclpy.init(args=args)
    node = ArucoStaticFramePubliser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

main()