#!/usr/bin/env python
"""
Created on Wed Feb 27 12:32:07 2019

@author: mitchell
"""

import rospy
import std_msgs.msg
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2

class image_republish:
    def __init__(self):
        
        self.image_pub = rospy.Publisher("/camera/rgb/image_color",Image, queue_size=1)
        self.pointcloud_pub = rospy.Publisher("/camera/depth/points",PointCloud2, queue_size=1)        
        
        self.bridge = CvBridge()
        
        rospy.Subscriber("/camera/left/image_raw",Image,self.img_callback)
        rospy.Subscriber("/seikowave_node/cloud",PointCloud2,self.pointcloud_callback)        
        

        self.img = Image()     

    def img_callback(self, msg):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
          print(e)
          
        cv_image = cv2.flip(cv_image, -1)  
        try:
          self.img = self.bridge.cv2_to_imgmsg(cv_image, "mono8")
        except CvBridgeError as e:
          print(e)        
        

    def pointcloud_callback(self, msg):
        self.image_pub.publish(self.img)
        self.pointcloud_pub.publish(msg)


    def run(self):
        r = rospy.Rate(10)
        i = 0
        while not rospy.is_shutdown():
            i+=1
            r.sleep()

if __name__ == '__main__':
    rospy.init_node("image_sync")
    IR = image_republish()
    rospy.spin()
