#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as Image2
from visualization_msgs.msg import Marker,MarkerArray
from smoke_pkg.msg import Kitti_Detection_Format, Kitti_Detection_Array

import cv2
import sys
import yaml
import torch
import numpy as np
from smoke.config import cfg
from smoke_kitti_vis import *
from smoke.structures.image_list import to_image_list
from smoke.modeling.heatmap_coder import get_transfrom_matrix
from smoke.structures.params_3d import ParamsList
import torchvision.transforms.functional as TF

NODE_NAME = 'smoke_kernel'

TOPIC_NAME_RAW_IN = "/camera/image_raw"

TOPIC_CAMERA_INFO = "/camera/camera_info"

TOPIC_NAME_DETECTION = "/smoke/detect"


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


class Callback(object):
    """docstring for Callback"""
    def __init__(self, GPU_DEVICE, CPU_DEVICE, model, pub_detection):
        super(Callback, self).__init__()
        self.GPU_DEVICE = GPU_DEVICE
        self.CPU_DEVICE = CPU_DEVICE
        self.model = model
        self.pub_detection = pub_detection

    def callback_raw(self, data):
        cv_image     = imgmsg_to_cv2(data)
        img          = Image.fromarray(cv_image)
        center       = np.array([i / 2 for i in img.size], dtype=np.float32)
        size         = np.array([i for i in img.size], dtype=np.float32)
        center_size  = [center, size]
        trans_affine = get_transfrom_matrix(
            center_size,
            [cfg.INPUT.WIDTH_TRAIN, cfg.INPUT.HEIGHT_TRAIN]
        )
        trans_affine_inv = np.linalg.inv(trans_affine)
        img = img.transform(
            (cfg.INPUT.WIDTH_TRAIN, cfg.INPUT.HEIGHT_TRAIN),
            method=Image.AFFINE,
            data=trans_affine_inv.flatten()[:6],
            resample=Image.BILINEAR,
        )
        trans_mat = get_transfrom_matrix(
            center_size, 
            [cfg.INPUT.WIDTH_TRAIN  // cfg.MODEL.BACKBONE.DOWN_RATIO,
             cfg.INPUT.HEIGHT_TRAIN // cfg.MODEL.BACKBONE.DOWN_RATIO]
        )

        target = ParamsList(image_size = size, is_train = False)
        target.add_field("trans_mat", trans_mat)
        target.add_field("K", self.K)
        output = self.model(to_image_list(TF.to_tensor(img)).to(self.GPU_DEVICE),(target,))
        output = output.to(self.CPU_DEVICE)
        prediction = []
        VEHICLES = ["Car","Cyclist","Pedestrian"]
        
        kitti_detection_array = Kitti_Detection_Array()
        kitti_detection_array.header = Header(stamp = rospy.Time.now())

        for j,p in enumerate(output):
            p = p.detach().numpy()
            p = p.round(4)
            row = [int(p[0]),0, 0] + p[1:].tolist()
            prediction.append(row)      
            distance_z = row[13]
            kitti_detection = Kitti_Detection_Format(type=row[0],
                                        truncated = row[1],
                                        occluded = row[2],
                                        alpha = row[3],
                                        bbox = [row[4],row[5],row[6],row[7]],
                                        dimensions = [row[8],row[9],row[10]],
                                        location = [row[11],row[12],distance_z],
                                        rotation_y = row[14],
                                        score = row[15])
            kitti_detection_array.detection.append(kitti_detection)

        self.pub_detection.publish(kitti_detection_array)

    def callback_camera_info(self, data):
        self.K = np.array(data.K).reshape((3,3)).astype(np.float32)

def listener():
    cfg.merge_from_file("smoke_gn_vector.yaml")
    cfg.freeze()

    GPU_DEVICE = "cuda"
    CPU_DEVICE = torch.device("cpu")
    model = torch.load('model_og.pth')
    model = model.to(GPU_DEVICE)
    model.eval()
    print("#"*50)
    print("SMOKE model loaded to GPU!")

    rospy.init_node(NODE_NAME, anonymous=True)    

    pub_detection  = rospy.Publisher(TOPIC_NAME_DETECTION, Kitti_Detection_Array, queue_size = 0)

    callback_handler = Callback(GPU_DEVICE, CPU_DEVICE, model, pub_detection)

    camera_info_sub = rospy.Subscriber(TOPIC_CAMERA_INFO , CameraInfo, callback_handler.callback_camera_info)
    
    camera_sub  =  rospy.Subscriber(TOPIC_NAME_RAW_IN, Image2, callback_handler.callback_raw)

    rospy.spin()

if __name__ == '__main__':
    listener()