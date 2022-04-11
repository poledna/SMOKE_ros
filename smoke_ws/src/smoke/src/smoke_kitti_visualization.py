#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as Image2
from smoke_pkg.msg import Kitti_Detection_Format, Kitti_Detection_Array

from detectionInfo import detectionInfo
from PIL import Image as Image
from smoke_kitti_vis import *
import sys
import cv2

NODE_NAME = 'smoke_view'

TOPIC_NAME_RAW_IN = "/camera/image_raw"

TOPIC_CAMERA_INFO = "/camera/camera_info"

TOPIC_NAME_DETECTION_VIEW = "/smoke/detect"

TOPIC_NAME_DETECTION = "/smoke/detect/image"

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image2()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    # img_msg.data = cv_image.tostring()
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

def callback(img, info, data,publisher):
    
    K = np.array(info.K).reshape((3,3))
    P2 = np.array(info.P).reshape((3,4))
    prediction = []
    for detect in data.detection:
        prediction.append(detectionInfo(detect))

    opencv_image = draw_3Dbox(imgmsg_to_cv2(img), prediction, P2)
    img_detect = cv2_to_imgmsg(opencv_image)
    publisher.publish(img_detect)

def main():

    rospy.init_node(NODE_NAME, anonymous=True)    

    pub_detection  = rospy.Publisher(TOPIC_NAME_DETECTION, Image2, queue_size = 0)

    camera_info_sub = message_filters.Subscriber(TOPIC_CAMERA_INFO, CameraInfo)
    camera_sub  =  message_filters.Subscriber(TOPIC_NAME_RAW_IN, Image2)
    detect_sub  =  message_filters.Subscriber(TOPIC_NAME_DETECTION_VIEW, Kitti_Detection_Array)

    ts = message_filters.ApproximateTimeSynchronizer([camera_sub, camera_info_sub, detect_sub], 10, 1)
    ts.registerCallback(callback,pub_detection)

    rospy.spin()
    pass


if __name__ == '__main__':
    main()