#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 13:35:25 2019

@author: mitchell
"""
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pointcloud_converter
import open3d
import cv2
import time

class PTI:
    def __init__(self):
        self.pointcloud_x = []
        self.pointcloud_y = []
        self.pointcloud_z = []
        self.pointcloud_i = []
        
        rospy.init_node('pointcloud_to_image')
        rospy.Subscriber('/tof/points/raw', PointCloud2, self.pointcloud_callback)
        #rospy.spin()
        
    def point_to_pixel(self, px, py, min_x, min_y, step_x, step_y):
        pix_x = int(round((px-min_x)/step_x))
        pix_y = int(round((py-min_y)/step_y))
        
        return pix_x, pix_y
    
    def pointcloud_callback(self, data):
        #print(data.header)
        #img = np.zeros((200, 348))
        #pointcloud_x = []
        #pointcloud_y = []
        #pointcloud_z = []
        #pointcloud_i = []
        self.pointcloud_x = []
        self.pointcloud_y = []
        self.pointcloud_z = []
        self.pointcloud_i = []
        
        for point in pointcloud_converter.read_points(data):
            self.pointcloud_x.append(point[0])
            self.pointcloud_y.append(point[1])
            self.pointcloud_z.append(point[2])
            #self.intensity = point[3]
            #nan = np.isnan(intensity)
            #if nan:
                #intensity = np.nan_to_num(intensity)
            #pointcloud_i.append(intensity)
        
        
        """
        #intensity_max = max(pointcloud_i)
        #pointcloud_i = np.multiply(np.divide(pointcloud_i, intensity_max),255)

        max_x = max(pointcloud_x)
        min_x = min(pointcloud_x)
        size_x = max_x-min_x
        step_x = size_x/(200.0-1)
        
        max_y = max(pointcloud_y)
        min_y = min(pointcloud_y)
        size_y = max_y-min_y  
        step_y = size_y/(348.0-1)
        
        
        for i in range(0, len(pointcloud_x)):
            x = pointcloud_x[i]
            y = pointcloud_y[i]
            x_pix, y_pix = self.point_to_pixel(x,y,min_x,min_y,step_x,step_y)
            if i < len(pointcloud_i):
                   img[x_pix,y_pix] = pointcloud_i[i]     
                   
        cv2.imshow('img', img)      
        cv2.waitKey(1)             
        """
        
    def main(self):
        time_inital = time.time()
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            time_elapsed = time.time() - time_inital
            time_inital = time.time()
            pointcloud_x = self.pointcloud_x
            pointcloud_y = self.pointcloud_y
            pointcloud_z = self.pointcloud_z
            if len(self.pointcloud_x):
                max_x = max(pointcloud_x)
                min_x = min(pointcloud_x)
                size_x = max_x-min_x
                step_x = size_x/(200.0-1)
                
                max_y = max(pointcloud_y)
                min_y = min(pointcloud_y)
                size_y = max_y-min_y  
                step_y = size_y/(348.0-1)
                
                for i in range(0, min(len(pointcloud_x), len(pointcloud_y))):
                    x = pointcloud_x[i]
                    y = pointcloud_y[i]
                    x_pix, y_pix = self.point_to_pixel(x,y,min_x,min_y,step_x,step_y)       
            
            
            print(1/time_elapsed)
            r.sleep()
                
        
if __name__ == '__main__':
    pti = PTI()
    pti.main()




























