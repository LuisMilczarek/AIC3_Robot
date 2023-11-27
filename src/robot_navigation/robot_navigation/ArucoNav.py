#!/usr/bin/env python

import rclpy
import cv2 as cv
import numpy as np

from copy import deepcopy

from rclpy.node import Node
from rclpy.duration import Duration
from robot_vision_msgs.msg import ArucoRecognitions, ArucoDetection
from tf2_py import TransformException
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_vector3
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped, PoseWithCovarianceStamped
from message_filters import TimeSynchronizer, Subscriber


class ArucoNav(Node):
    def __init__(self) -> None:
        super().__init__("aruco_nav")
        self.recognitions_sub = self.create_subscription(ArucoRecognitions, "/robot_vision/ar/recognitions", self.callback, qos_profile=1)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.update_amcl, qos_profile=1)
        self.debug_pub = self.create_publisher(MarkerArray,"nav/debug",1)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,"/initialpose",1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self,spin_thread=True)
        self.get_logger().info("started")
        self.amcl_position = None
        self.amcl_orientation = None

    
    def callback(self, msg :  ArucoRecognitions) -> None:
        det : ArucoDetection
        debug_markers = MarkerArray()
        det : ArucoDetection
        xs = []
        ys = []
        # zs = []
        # rs = []
        # ps = []
        yas = []

        for det in msg.detections:
            # self.get_logger().error(f"{det.pose.pose.position.x},{det.pose.pose.position.y},{det.pose.pose.position.z}")
            try:
                transform = self.tf_buffer.lookup_transform( "map",f"marker_{det.id:04d}", rclpy.time.Time())
            except TransformException as e:
                self.get_logger().warn(f"Could Not get transform from marker_{det.id:04d}!! : {e}")
                continue
            # pose = do_transform_pose_stamped(det.pose, transform)
            # pose = det.pose
            # self.get_logger().error(f"{pose}\n")
            vec = Vector3Stamped()
            vec.header.frame_id = "base_footprint"
            vec.header.stamp = det.pose.header.stamp
            vec.vector.x = 0.0
            vec.vector.y = 0.0
            vec.vector.z = 0.0
            # new_vec = do_transform_vector3(vec, transform)

            # self.get_logger().warn(f"AAAAA:")



            rvec = euler_from_quaternion([
                det.pose.pose.orientation.x,
                det.pose.pose.orientation.y,
                det.pose.pose.orientation.z,
                det.pose.pose.orientation.w
            ])
            tvec = np.array(
                [
                    [det.pose.pose.position.x],
                    [det.pose.pose.position.y],
                    [det.pose.pose.position.z]
                ])
            rotM = cv.Rodrigues(np.array(rvec))[0]
            position = -np.matrix(rotM).T * np.matrix(tvec)
            # self.get_logger().info(f"{position}")

            # self.get_logger().error(f"Original: {rvec}")
            q = quaternion_from_euler(rvec[0],rvec[1],-1*rvec[2]-np.pi)

            pose = Pose()
            pose.position.x
            pose.position.x = position[0,0]
            pose.position.y = position[1,0]
            pose.position.z = position[2,0]
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            poseS = PoseStamped()
            poseS.pose = pose
            poseS.header = det.pose.header
            
            map_pose = do_transform_pose_stamped(poseS, transform)
            xs.append(map_pose.pose.position.x)
            ys.append(map_pose.pose.position.y)
            # zs.append(map_pose.pose.position.z)

            (_,_,y) = euler_from_quaternion((map_pose.pose.orientation.x,map_pose.pose.orientation.y,map_pose.pose.orientation.z,map_pose.pose.orientation.w))
        
            yas.append(y)
            # self.get_logger().warn(f"{map_pose}")



            #DEBUG MARKER

            marker = Marker()
            marker.header = det.pose.header
            marker.header.frame_id = f"marker_{det.id:04d}"
            marker.id = 255+det.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose = deepcopy(pose)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 255.0
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=10).to_msg()
            debug_markers.markers.append(marker)


        if len(xs)>0 and len(ys)>0 and len(yas)>0:
            final_pose = Pose()
            final_pose.position.x = np.median(xs)
            final_pose.position.y = np.median(ys)
            final_pose.position.z = 0.0
            
            roll = 0.0
            pitch = 0.0
            yaws = np.median(yas)

            dist_xy = float("inf")
            if self.amcl_position is not None and self.amcl_orientation is not None:
                dist_xy = np.linalg.norm(np.array([final_pose.position.x,final_pose.position.y])-self.amcl_position)

            if dist_xy >= 1.0:
                self.get_logger().error("ENTREI")
                q = quaternion_from_euler(roll, pitch, yaws)

                final_pose.orientation.x = q[0]
                final_pose.orientation.y = q[1]
                final_pose.orientation.z = q[2]
                final_pose.orientation.w = q[3]

                marker = Marker()
                marker.header = det.pose.header
                marker.header.frame_id = f"map"
                marker.id = 255+det.id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose = deepcopy(final_pose)
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                marker.color.r = 0.0
                marker.color.g = 255.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime = Duration(seconds=10).to_msg()
                debug_markers.markers.append(marker)

                initial_pose = PoseWithCovarianceStamped()
                initial_pose.header.frame_id = "map"
                initial_pose.header.stamp = self.get_clock().now().to_msg()
                initial_pose.pose.pose = final_pose
                self.pose_pub.publish(initial_pose)


            
            self.get_logger().warn(f"{final_pose.position.x},{final_pose.position.y}, Q: {q[2]} , {q[3]}")

        self.debug_pub.publish(debug_markers)
        return
    
    def update_amcl(self, msg : PoseWithCovarianceStamped) -> None:
        self.amcl_position = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])
        _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.amcl_orientation = yaw
        return

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"{e}")
    node.destroy_node()
    rclpy.shutdown()