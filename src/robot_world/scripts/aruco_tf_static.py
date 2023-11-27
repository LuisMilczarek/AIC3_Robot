#!/usr/bin/env python
import os
import rclpy
import numpy as np
from time import sleep

from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

class ArucoStaticFramePubliser(Node):
    def __init__(self) -> None:
        super().__init__("aruco_static_transform_publisher")
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.gazebo_spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.world_models = os.path.join(get_package_share_directory("robot_world"),"models",'arucos')
        self.spawn_arucos()
        self.make_transforms()
    
    def spawn_arucos(self) -> None:
        while not self.gazebo_spawn_client.wait_for_service(10):
            self.get_logger().warn("Wainting for spawn entity")
        q = quaternion_from_euler(0, np.pi/2, 0)
        width = 20
        for i in range(220):
            f = open(os.path.join(self.world_models,f'marker_{i:04d}','model.sdf'),'r')
            xml = f.read()
            req = SpawnEntity.Request()
            req.name = f"marker_{i}"
            req.initial_pose.position.x = float(i//width)-4-.25
            req.initial_pose.position.y = float(i%width)-10
            req.initial_pose.position.z = 4.0
            req.initial_pose.orientation.x = q[0]
            req.initial_pose.orientation.y = q[1]
            req.initial_pose.orientation.z = q[2]
            req.initial_pose.orientation.w = q[3]

            req.xml = xml

            future = self.gazebo_spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self,future=future)
            # self.get_logger().info(f"Marker {i} result: {future.result()}")

    def make_transforms(self) -> None:
        width=20
        for i in range(200):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = f"marker_{i:04d}"

            #  pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            #     'y': LaunchConfiguration('y_pose', default='0.00'),
            #     'z': LaunchConfiguration('z_pose', default='4.00'),
            #     'R': LaunchConfiguration('roll', default='0.00'),
            #     'P': LaunchConfiguration('pitch', default='1.57079632679'),
            #     'Y': LaunchConfiguration('yaw', default='0.00')}
            
            q = quaternion_from_euler(0, 0, 3*np.pi/2)

            t.transform.translation.x = float(i//width)-4
            t.transform.translation.y = float(i%width)-10
            t.transform.translation.z = 4.0

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_static_broadcaster.sendTransform(t)
        # rclpy.spin_once(self)

        self.get_logger().info("Static Transform Published")

        

def main(args=None):
    rclpy.init(args=args)
    node = ArucoStaticFramePubliser()
    r = node.create_rate(60)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

main()