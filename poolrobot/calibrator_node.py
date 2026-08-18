from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import rclpy
import yaml
import math
import csv
import os
import cv2
from cv_bridge import CvBridge

#This needs to 
#- subscribe to /camera/camera/color/image_raw
#- read the calibration data from the YAML file
#- publish the calibration data as a /camera_info topic with the same timestamp as /camera/camera/color/image_raw

class CalibratorNode(Node):
    def __init__(self):
        super().__init__('calibrator_node')
        self.bridge = CvBridge()
        self.declare_parameter('calyaml_path', '')
        calyaml_path = self.get_parameter('calyaml_path').get_parameter_value().string_value
        self.camera_info_msg = self.get_calibration(calyaml_path)

        #Create subscriber looking for /tf, every time the topic is published tf_callback is called
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )

        #Create publisher to publish camera info intrinsics
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/my_camera_info',
            10
        )

        #Create publisher to publish image raw
        self.image_raw_pub = self.create_publisher(
            Image,
            '/my_image_rect',
            10
        )


    def image_callback(self,msg):

        #Set header and frame ID from camera_info and image_raw to match.
        self.camera_info_msg.header.stamp = msg.header.stamp
        self.camera_info_msg.header.frame_id = msg.header.frame_id

        #Convert image to OpenCV image for processing.
        cv_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #Store K and D matrices of calibration
        K = np.array(self.camera_info_msg.k).reshape(3,3)
        D = np.array(self.camera_info_msg.d)

        #Undistort the image using the calibration data
        cv_rect = cv2.undistort(cv_raw,K,D)

        #Convert back to ROS topic message
        rect_msg = self.bridge.cv2_to_imgmsg(cv_rect, encoding='bgr8')
        rect_msg.header = msg.header

        #Publish the calibration data as a /camera_info topic with the same timestamp as /camera/camera/color/image_raw
        self.camera_info_pub.publish(self.camera_info_msg)
        #Publish the image raw data
        self.image_raw_pub.publish(rect_msg)


    #Parses a camera calibration YAML file and returns a sensor_msgs/CameraInfo object.
    def get_calibration(self, calyaml_path: str) -> CameraInfo:
        
        with open(calyaml_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        camera_info = CameraInfo()

        # Image dimensions
        camera_info.width = int(calib_data['image_width'])
        camera_info.height = int(calib_data['image_height'])

        # Distortion Model
        camera_info.distortion_model = str(calib_data['distortion_model'])

        # Matrices and Coefficients (converted to float lists)
        camera_info.d = [float(x) for x in calib_data['distortion_coefficients']['data']]
        camera_info.k = [float(x) for x in calib_data['camera_matrix']['data']]
        camera_info.r = [float(x) for x in calib_data['rectification_matrix']['data']]
        camera_info.p = [float(x) for x in calib_data['projection_matrix']['data']]

        return camera_info

       
#MAIN FUNCTION ======================================================================
def main(args = None):
    rclpy.init(args = args)
    node = CalibratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
