#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as Image2
from std_msgs.msg import Header
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

def callback_raw(data, args):
    model, K, GPU_DEVICE, CPU_DEVICE, pub_kitti_detection = args
    
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
    target.add_field("K", K)
    output = model(to_image_list(TF.to_tensor(img)).to(GPU_DEVICE),(target,))
    output = output.to(CPU_DEVICE)
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

    pub_kitti_detection.publish(kitti_detection_array)


def _resize_intrinsics(input_size, target_shape):
    resize_matrix = np.eye(3)
    resize_matrix[0, 0] = float(target_shape[1])/float(input_size[1])
    resize_matrix[1, 1] = float(target_shape[0])/float(input_size[0])
    return resize_matrix

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

    intrinsic_matrix = K = np.array([[1942.94197430278,0,0],[0,1941.98949906699,0],[961.402331648884,587.317863907068,1]]).T.astype(np.float32)

    NODE_NAME = 'camera_SMOKE'

    TOPIC_NAME_RAW_IN = "/camera/image_raw"
    
    TOPIC_NAME_DETECTION = "/smoke/detect"

    rospy.init_node(NODE_NAME, anonymous=True)    

    pub_detection  = rospy.Publisher(TOPIC_NAME_DETECTION, Kitti_Detection_Array, queue_size = 0)

    camera_sub  =  rospy.Subscriber(TOPIC_NAME_RAW_IN, Image2, callback_raw,
                                                              (model,K,
                                                               GPU_DEVICE,CPU_DEVICE,
                                                               pub_detection))

    rospy.spin()

if __name__ == '__main__':
    listener()

