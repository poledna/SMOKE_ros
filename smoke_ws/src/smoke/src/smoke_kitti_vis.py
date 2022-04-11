import numpy as np
import argparse
import os
import cv2
from math import degrees
from PIL import Image
from detectionInfo import detectionInfo

VEHICLES = ['Car', 'Cyclist', 'Pedestrian']

def pairs(seq):
    i = iter(seq)
    prev = next(i)
    for item in i:
        yield prev, item
        prev = item

def get_color(prediction):
    color = (0,255,0) #carro
    color_text = (0,125,0)
    if prediction.name == 1:
        color = (255,255,0) #ciclista 
        color_text = (127,125,0)
    elif prediction.name == 2: 
        color = (0,255,255) #pedestre
        color_text = (0,125,127)
    return color, color_text

def draw_3Dbox(img, line, P2):
    size = img.size 
    for obj in line:
        thickness = 1
        corners_2D = compute_3Dbox(P2, obj)
        color, color_text = get_color(obj)


        corners_2D = corners_2D.astype(np.int16)
        shapes=np.zeros_like(img,np.uint8)

        cv2.rectangle(shapes,corners_2D[:, 1],(corners_2D[:, 3][0],corners_2D[:, 3][1]),color, cv2.FILLED)
        cv2.rectangle(img,corners_2D[:, 1],(corners_2D[:, 3][0],corners_2D[:, 3][1]),color, thickness)
        
        out = img.copy()
        alpha = 0.5
        mask = shapes.astype(bool)
        out[mask]=cv2.addWeighted(img,alpha,shapes,1-alpha,0)[mask]
        img =out.copy()

        img = cv2.rectangle(img,corners_2D[:, 0],(corners_2D[:, 6][0],corners_2D[:, 6][1]),color,thickness)
        
        img = cv2.line(img,corners_2D[:, 1], corners_2D[:, 0],color,thickness)
        img = cv2.line(img,corners_2D[:, 2], corners_2D[:, 7],color,thickness)
        img = cv2.line(img,corners_2D[:, 3], corners_2D[:, 6],color,thickness)
        img = cv2.line(img,corners_2D[:, 4], corners_2D[:, 5],color,thickness)

        detect_text = f"{VEHICLES[obj.name]}:{int(obj.score*100)}%"
        (w, h), _ = cv2.getTextSize(detect_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        img = cv2.rectangle(img, (obj.xmin, (obj.ymin - 20)), ((obj.xmin + w), obj.ymin) , color, -1)
        img = cv2.putText(img, detect_text, (obj.xmin, obj.ymin ),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,color_text, 1)
    return img 


def compute_3Dbox(P2, obj):
    # Draw 3D Bounding Box
    R = np.array([[np.cos(obj.rot_global), 0, np.sin(obj.rot_global)],
                  [0, 1, 0],
                  [-np.sin(obj.rot_global), 0, np.cos(obj.rot_global)]])
    

    x_corners = [0, obj.l, obj.l, obj.l, obj.l, 0, 0, 0]  # -l/2
    y_corners = [0, 0, obj.h, obj.h, 0, 0, obj.h, obj.h]  # -h
    z_corners = [0, 0, 0, obj.w, obj.w, obj.w, obj.w, 0]  # -w/2

    x_corners = [i - obj.l / 2 for i in x_corners]
    y_corners = [i - obj.h for i in y_corners]
    z_corners = [i - obj.w / 2 for i in z_corners]

    corners_3D = np.array([x_corners, y_corners, z_corners])
    corners_3D = R.dot(corners_3D)
    corners_3D += np.array([obj.tx, obj.ty, obj.tz]).reshape((3, 1))

    corners_3D_1 = np.vstack((corners_3D, np.ones((corners_3D.shape[-1]))))
    corners_2D = P2.dot(corners_3D_1)
    corners_2D = corners_2D / corners_2D[2]
    corners_2D = corners_2D[:2]
    return corners_2D


