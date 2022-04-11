#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image  
import cv2
import yaml
from sensor_msgs.msg import CameraInfo
from time import sleep

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data

    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

rospy.init_node("send_image")

publisher_cam_raw = rospy.Publisher("/camera/image_raw/",Image,queue_size=10)

image = cv2.imread('kitti_images/001000.png')

publisher_cam_info = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=10)

camera_info_msg = yaml_to_CameraInfo("ost.yaml")

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    # img_msg.data = cv_image.tostring()
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg


while not rospy.is_shutdown():
    publisher_cam_info.publish(camera_info_msg)
    publisher_cam_raw.publish(cv2_to_imgmsg(image))
    sleep(1)
