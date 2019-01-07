#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan  4 09:16:15 2019

@author: mitchell
"""
import logging
import rospy
import numpy as np
import math
import cv2
import sensor_msgs.point_cloud2 as pointcloud_converter
#import pcl

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TOFCalibration:
    
    def __init__(self):
        self.image_points_x = []
        self.image_points_y = []
        self.pointcloud_points_x = []
        self.pointcloud_points_y = []
        self.pointcloud_points_z = []
        #self.pointcloud = []
        self.img = np.zeros((0,0,3), np.uint8)
        self.bridge = CvBridge()   
        
        self.equal = False
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        #rospy.Subscriber('/camera/rgb/image_color', Image, self.img_callback)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pointcloud_callback)
        
    def pointcloud_callback(self, data):
        for point in pointcloud_converter.read_points(data):
            if not math.isnan(point[0]) and not math.isnan(point[1]) and not math.isnan(point[2]):
                self.pointcloud_points_x.append(float(point[0]))
                self.pointcloud_points_y.append(float(point[1]))
                self.pointcloud_points_z.append(float(point[2]))
        
    
    def img_callback(self, data):
        try:
          self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)        
        cv2.setMouseCallback('image',self._mouse_click)
        cv2.imshow("image", self.img)
        cv2.waitKey(3)
        
    def main(self):
        r = rospy.Rate(60)
        while not rospy.is_shutdown():
            x = self.pointcloud_points_x[:-4]
            y = self.pointcloud_points_y[:len(x)]
            z = self.pointcloud_points_z[:len(x)]
        
            equal = len(x) == len(y) == len(z)
            if equal:
                print(len(x), len(y), len(z))
                self.ax.scatter(x,y,z)
                
                
                """
                try:
                    self.ax.scatter(x, y, z)
                except Exception as e:
                    logging.error('Can not plot: '+ str(e))
                """
            #plt.pause(5)
            
            r.sleep()
    
    
    def _mouse_click(self,event,x,y,flags,param):
       """
       Callback function for mouse click event on image frame
       
       Places clicked points into x1_ and y1_points lists
       """
       if event == cv2.EVENT_LBUTTONDOWN:
           self.image_points_x.append(x)
           self.image_points_y.append(y)
           #Draw circle where clicked
           cv2.circle(self.img,(x,y), 5, (255,0,0), -1)
           cv2.imshow('image', self.img)
           cv2.waitKey(1000)
           
           
    




if __name__ == '__main__':
    rospy.init_node('tof_camera_calibration')
    cal = TOFCalibration()
    cal = cal.main()
    
    #rospy.spin()
    