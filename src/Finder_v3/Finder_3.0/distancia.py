#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, os.path, rospy
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import *
from math import *
from sympy import *
from time import sleep
from sensor_msgs.msg import Image as ImageMsg
#from people_msgs.msg import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from dynamic_reconfigure.parameter_generator_catkin import *

exis = Symbol('exis')
bridge = CvBridge()
id_person = 0
detected_objects = []

class finder_v2dot0:

  def __init__(self):
    yolo_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes , self.callback_box, queue_size=10)
  
  def callback_box(self, data):
    os.system('cls' if os.name == 'nt' else 'clear')

    self.lmax_person = []
    self.lmin_person = []
    self.x_center = []
    self.y_center = []
    #self.probab_person = []
    self.classe = []
    self.dist_final = []
    self.dist_x = []
    self.dist_y = []
    self.X_map = []
    self.Y_map = []
    dist = []
    X = []
    Y = []
    Z = []

    for f in range(len(data.bounding_boxes)):
        self.classe.append(data.bounding_boxes[f].Class)
        self.x_center.append((data.bounding_boxes[f].xmax + data.bounding_boxes[f].xmin)/2)
        self.y_center.append((data.bounding_boxes[f].ymax - data.bounding_boxes[f].ymin)/2)
        print("X_med = {}" .format(self.x_center[f]))
        print("Y_med = {}" .format(self.y_center[f]))

        self.df = 216.3355 # Distância Focal
        self.cx = 320      # Resolução da imagem em x / 2
        self.cy = 240      # Resolução da imagem em y / 2
           

        dist.append(depth_frame[int(self.y_center[0]), int(self.x_center[0])])
        X.append(dist[f]*(self.x_center[f] - self.cx)/self.df)
        Y.append(dist[f]*(self.y_center[f] - self.cy)/self.df)
        Z.append(dist[f])
        print("Distnacia = {} m" .format(Z[f]))


def callback_depth(data):
  global depth_frame 
  depth_frame = bridge.imgmsg_to_cv2(data, "passthrough")

if __name__ == '__main__':
  rospy.init_node('finder_alpha')
  os.system('cls' if os.name == 'nt' else 'clear')
  sleep(1.00)
  
  finder_v2dot0()
  rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", ImageMsg, callback_depth)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down...")
