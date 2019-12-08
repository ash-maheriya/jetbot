#!/usr/bin/env python

import cv2 as cv
#import imutils
import numpy as np
import pdb
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import sys
import os
dir = os.environ['HOME'] + '/work/tf-infer/utils/'
sys.path.insert(0, dir)
import mod_trt_inference as infer

class color:

    def __init__(self):
        self.imagePush = None
        self.cmd = Twist()
        self.cam = cv.VideoCapture(1)
        self.ret = None
        self.frame = None
        self.drive_pub = rospy.Publisher("/jetsoncar_teleop_joystick/cmd_vel", Twist, queue_size = 1)

    def color_drive(self):
        print("running")
#        low_bound = [45, 52, 25]   # doesn't work
#        high_bound = [94, 255, 127]# doesn't work
#        low_bound = [55, 22, 31]       # upstairs room
#        high_bound = [84, 244, 151]    # upstairs room
        low_bound = [75, 67, 138]      # downstairs living room
        high_bound = [98, 124, 255]   # downstairs living room

        self.ret, self.frame = self.cam.read()
        if (self.frame) is None:
            time.sleep(0.5)
            print("sleeping")
            return None
        
         
        box, self.imagePush = self.read_color(self.frame, low_bound, high_bound, True)
        midpoint = 336
        line = (abs(box[0][0] - box[1][0])/2.) + box[0][0]# - 72
        shift = midpoint - line
        kpconst = 300.
        self.cmd.angular.z = shift/kpconst
        area = (box[0][0] - box[1][0]) * (box[0][1] - box[1][1])
        print("area: " + str(area))  # if the box is a meter away, the area is about 2500      
        
        speed = ((area/2500.) - 1) * - 1
        if speed > 1:
            speed = 1
        if speed < -1:
            speed = -1
        print("speed: " + str(speed))
        
        
        self.cmd.linear.x = speed
        self.drive_pub.publish(self.cmd)

    def read_color(self, img, low_bound, high_bound, show_image = False):
        new_img = cv.cvtColor(img, cv.COLOR_RGB2HSV)

        low_range = np.array(low_bound)
        high_range = np.array(high_bound)

        hue, sat, val = cv.split(new_img)
        hue_threshed = cv.inRange(hue, low_bound[0], high_bound[0]) 
        sat_threshed = cv.inRange(sat, low_bound[1], high_bound[1]) 
        val_threshed = cv.inRange(val, low_bound[2], high_bound[2]) 

       # mask = cv.inRange(new_img, low_range, high_range)

       # filtered = cv.bitwise_and(new_img, new_img, mask = mask)

        threshed_image = cv.multiply(sat_threshed, hue_threshed)
        threshed_image = cv.multiply(threshed_image, val_threshed)

        
        _, contours, heirarchy = cv.findContours(threshed_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key = cv.contourArea, reverse = True)
        x1, y1, x2, y2 = 0, 0, 0, 0

        if len(contours) != 0:
            contours_max = max(contours, key = cv.contourArea)

            x1, y1, x2, y2 = cv.boundingRect(contours_max)

            cv.rectangle(img, (x1, y1), (x1 + x2, y1 + y2), (0, 255, 0), 2)

        if show_image:
            cv.imshow("Color segmentation", img)
            cv.imshow("threshed", threshed_image)
            if cv.waitKey(1) & 0xFF == ord('q'):
                cv.destroyAllWindows()

        return ((x1, y1), (x1 + x2, y1 + y2)), img


rospy.init_node("ballDetector")
rate = rospy.Rate(60)
bD = color()

while not rospy.is_shutdown():
    bD.color_drive()
 #   rospy.spin()


