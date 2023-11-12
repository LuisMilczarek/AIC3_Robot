#!/usr/bin/env python3
import rclpy
import cv2 as cv
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from robot_vision_msgs.msg import ArucoRecognitions, ArucoDetection

from tf_transformations import quaternion_from_euler

class ArucoRecognition(Node):
    def __init__(self) -> None:
        super().__init__("ArucoRecognition")
        self.subscriber = self.create_subscription(Image, "/camera/image_raw",self.image_callback,1)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info",self.get_camera_info,1)
        self.debug_publisher = self.create_publisher(Image, "/robot_vision/ar/debug", 1)
        self.aruco_publisher = self.create_publisher(ArucoRecognitions, "/robot_vision/ar/recognitions", 1)
        self.bridge = CvBridge()
        self.i = 0
        self.parameters = cv.aruco.DetectorParameters()
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        self.dectector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.cam_mat = None
        self.dist_coef = None
        self.run = False

        marker_size = 1

        # self.marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
        #                       [marker_size / 2, marker_size / 2, 0],
        #                       [marker_size / 2, -marker_size / 2, 0],
        #                       [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        
        self.marker_points = np.array([[marker_size / 2, marker_size / 2, 0],
                              [-marker_size / 2, marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

    def image_callback(self, img : Image) -> None:
        if not self.run:
            return

        # self.get_logger().info(f"Recebido {self.i}, Formato: {img.encoding}")
        color_img = self.bridge.imgmsg_to_cv2(img)
        gray_img = cv.cvtColor(color_img, cv.COLOR_RGB2GRAY)
        corners, ids, rejected_img_points = self.dectector.detectMarkers(gray_img)
        # self.get_logger().info(f"Corners :{corners}, Ids: {ids}, rejected: {rejected_img_points}")

        if ids is not None:
            recognitions = ArucoRecognitions()
            recognitions.size = len(ids[0])
            # self.get_logger().error(f"IDS: {ids}")
            for c , id in zip(corners, ids[0]):
                detection = ArucoDetection()
                _, rVec, tVec = cv.solvePnP(self.marker_points, c, self.cam_mat, self.dist_coef)
                tVec = tVec[:,0]
                rVec = rVec[:,0]
                # print(f"rVec: {rVec}, tVec: {tVec}")
                detection.id = int(id)
                detection.pose.header = img.header
                
                # self.get_logger().error(f"THIS ID: {tVec}")
                detection.pose.pose.position.x = tVec[0]
                detection.pose.pose.position.y = tVec[1]
                detection.pose.pose.position.z = tVec[2]
                # self.get_logger().error(f"RVEC {rVec[:,0]}")

                q = quaternion_from_euler(*rVec)

                detection.pose.pose.orientation.x = q[0]
                detection.pose.pose.orientation.y = q[1]
                detection.pose.pose.orientation.z = q[2]
                detection.pose.pose.orientation.w = q[3]
                recognitions.detections.append(detection)

                cv.drawFrameAxes(color_img,self.cam_mat, self.dist_coef,rVec,tVec,1.0)
                # self.get_logger().info("HI")
            self.aruco_publisher.publish(recognitions)
        self.debug_publisher.publish(self.bridge.cv2_to_imgmsg(color_img,encoding=img.encoding,header=img.header))
        

        self.i += 1

    def get_camera_info(self, msg : CameraInfo) -> None:
        self.cam_mat = np.array(msg.k).reshape((3,3))
        self.dist_coef = np.array(msg.d)
        # self.get_logger().info(f"Cam mat: {self.cam_mat}, dist coef: {self.dist_coef}")
        self.camera_info_sub.destroy()
        self.run = True
    


def main(args=None):
    rclpy.init(args=args)
    node = ArucoRecognition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()