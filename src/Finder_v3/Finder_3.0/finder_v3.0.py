#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, os
import math, os, os.path, rospy
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import *
from math import *
from sensor_msgs.msg import LaserScan
from sympy import *
from time import sleep
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image as ImageMsg
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from msgs.msg import People, Person
from geometry_msgs.msg import PoseWithCovarianceStamped
from dynamic_reconfigure.parameter_generator_catkin import *
from visualization_msgs.msg import MarkerArray, Marker

exis = Symbol('exis')
bridge = CvBridge()
id_person = 0
shadow_pos_x = []
shadow_pos_y = []

robot_pose = [0, 0, 0]
robot_orientation = [0, 0, 0, 0]
detected_objects = []

class finder_v2dot0:

  def __init__(self):
    amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.callback_amcl, queue_size=10)
    yolo_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes , self.callback_box, queue_size=10)
    self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser, queue_size=10)

  def callback_amcl(self, data):
    global robot_pose, robot_orientation
    robot_pose = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
    robot_orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

  def callback_count(self, msg):
    self.counter = msg.count

  def callback_box(self, data):
    os.system('cls' if os.name == 'nt' else 'clear')
    print('\033[1;30;47m=' * 84)
    print("\033[1;30;47m                                     FINDER v2.1™")
    print('\033[1;30;47m=' * 84)
    print("\n\033[1;34;47mYOLOV3 is detecting {} object(s)...".format(len(data.bounding_boxes)))

    self.lmax_person = []
    self.lmin_person = []
    self.x_center = []
    self.y_center = []
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
    Y_robot = []
    X_robot = []
    
    numero = len(data.bounding_boxes) - 1

    for f in range(len(data.bounding_boxes)):
        self.classe.append(data.bounding_boxes[f].Class)

        if (self.classe[f] == "Static Object - Inanimate Being"):
          self.x_center.append((data.bounding_boxes[f].xmax - data.bounding_boxes[f].xmin)/2 + data.bounding_boxes[f].xmin)
          self.y_center.append((data.bounding_boxes[f].ymax - data.bounding_boxes[f].ymin)/2 + data.bounding_boxes[f].ymin)

        else:
          self.x_center.append((data.bounding_boxes[f].xmax - data.bounding_boxes[f].xmin)/2 + data.bounding_boxes[f].xmin)
          self.y_center.append((data.bounding_boxes[f].ymax - data.bounding_boxes[f].ymin)/2 + data.bounding_boxes[f].ymin)

        self.df = 216.3355 # Distância Focal
        self.cx = 320      # Resolução da imagem em x / 2
        self.cy = 240      # Resolução da imagem em y / 2
        
        dist.append(depth_frame[int(self.y_center[f]), int(self.x_center[f])])
        X.append(dist[f]*(self.x_center[f] - self.cx)/self.df)
        Y.append(dist[f]*(self.y_center[f] - self.cy)/self.df)
        Z.append(dist[f])
        print(f" : {data.bounding_boxes[f].Class}, {Z[f]}")

        # CONTROLA O ANGULO DA CAMERA
        camera_angle = 0 # -3.14 a 3.14

        # CALCULA A POSICAO DO ROBO
        q0 = robot_orientation[0]
        q1 = robot_orientation[1]
        q2 = robot_orientation[2]
        q3 = robot_orientation[3]

        yaw = math.atan2(2*(q0*q1 + q2*q3),(q0**2 - q1**2 - q2**2 + q3**2))
        yaw = -yaw - math.pi/2

        # CONVERTE PARA COORDENADAS DO ROBO
        Y_robot.append(-(X[f] *math.cos(camera_angle) - Z[f] *math.sin(camera_angle)))
        X_robot.append(X[f] *math.sin(camera_angle) + Z[f] *math.cos(camera_angle))

        # CONVERTE PARA COORDENADAS DO MAPA
        yaw_robot = -math.pi/2 - yaw

        self.X_map.append(X_robot[f] *math.cos(yaw_robot) - Y_robot[f] *math.sin(yaw_robot) + robot_pose[0])
        self.Y_map.append(X_robot[f] *math.sin(yaw_robot) + Y_robot[f] *math.cos(yaw_robot) + robot_pose[1])

  def callback_laser(self, msg):

    def people_func(nome, X_map, Y_map, xx):
      
      person = Person()
      person.name = nome

      person.pose.position.x = X_map
      person.pose.position.y = Y_map
      person.pose.position.z = 0

      person.proxemic.spread.x = xx
      person.proxemic.spread.y = xx
      person.proxemic.spread.z = 0
      person.pose.orientation.w = 1

      person.proxemic.freeBorder = 30
      person.proxemic.lethalBorder = 68

      people.people.append(person)

    def marker_func(object_type, red, green, blue, lmax, lmin, id_type):
      marker = Marker()
      marker.header.stamp = rospy.Time.now()
      marker.header.frame_id = "odom"
      marker.ns = object_type
      marker.type = 3
      marker.action = marker.ADD
      marker.scale.x = 0.50
      marker.scale.y = 0.50
      marker.scale.z = 0.50
      marker.color.a = 0.85
      marker.color.r = red
      marker.color.g = green
      marker.color.b = blue
      marker.lifetime = rospy.Duration(0.30)
      marker.pose.orientation.w = 0
      marker.pose.position.x = lmax
      marker.pose.position.y = lmin
      marker.pose.position.z = 0.0
      global id_person
      for o in markerArray.markers:
          o.id = id_type
      markerArray.markers.append(marker)

    marker_pub = rospy.Publisher("/marker_loc", MarkerArray, queue_size=10)
    people_pub = rospy.Publisher("/people", People, queue_size=5)
    rate = rospy.Rate(2) # 10h

    markerArray = MarkerArray()
    people = People()
    people.header.frame_id = 'map'
    people.header.stamp = rospy.Time.now()

    for l in range(len(self.classe)):
      global id_person
      id_person += 2

      if (self.classe[l] == "Dynamic Object - Living Being"):
        people_func(self.classe[l], self.X_map[l], self.Y_map[l], 0.8)
        marker_func('object_person', 1.00, 0.00, 0.00, self.X_map[l], self.Y_map[l], id_person)

      if (self.classe[l] == "Static Object - Inanimate Being"):
        people_func(self.classe[l], self.X_map[l], self.Y_map[l], 0.7)
        marker_func('object_person', 1.00, 1.00, 0.00, self.X_map[l], self.Y_map[l], id_person)

      if (self.classe[l] == "Dynamic Object - Inanimate Being"):
        people_func(self.classe[l], self.X_map[l], self.Y_map[l], 1)
        marker_func('object_person', 0.00, 1.00, 0.00, self.X_map[l], self.Y_map[l], id_person)
        
      if (self.classe[l] == "Static Object - Living Being"):
        people_func(self.classe[l], self.X_map[l], self.Y_map[l], 0.3)
        marker_func('object_person', 0.00, 1.00, 0.00, self.X_map[l], self.Y_map[l], id_person)

    end_time = time.time() + 1
    countTimer = 0.00
    sleepTime = 0.50
    while time.time() < end_time:
      time.sleep(sleepTime)
      people_pub.publish(people)
      marker_pub.publish(markerArray)
      rospy.sleep(0.0001)

def callback_depth(data):
  global depth_frame 
  depth_frame = bridge.imgmsg_to_cv2(data, "passthrough")

if __name__ == '__main__':
  rospy.init_node('finder_alpha')
  os.system('cls' if os.name == 'nt' else 'clear')
  print('\033[1;30;47m=' * 84)
  print("\033[1;30;47m                                     FINDER v2.1™")
  print('\033[1;30;47m=' * 84)
  sleep(1.00)
  
  finder_v2dot0()
  rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", ImageMsg, callback_depth)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down...")