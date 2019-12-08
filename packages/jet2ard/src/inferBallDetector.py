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

class InferBall:

    def __init__(self):
        self.imagePush = None
        self.cmd = Twist()
        self.cam = cv.VideoCapture(1)
        self.ret = None
        self.frame = None
        self.drive_pub = rospy.Publisher("/jetsoncar_teleop_joystick/cmd_vel", Twist, queue_size = 1)
        infer.setup()

    def infer_drive(self):
        print("running")
        
        self.ret, self.frame = self.cam.read()
        if (self.frame) is None:
            time.sleep(0.5)
            print("sleeping")
            return None
        
        box = infer.inferenceBox(self.frame)
        mid = (box[3] + box[1])/2
        area = (box[2] - box[0]) * (box[3] - box[1]) * 10000           
        if area != 0:
            self.cmd.angular.z = ((mid*2) - 1) * -1
            speed = ((area/80.) - 1) * - 1  # WAS 130, REDUCED FURTHER BECAUSE THAT WAS TOO CLOSE
        else:
            self.cmd.angular.z = 0
            speed = 0
        print("area: " + str(area))  
        print("mid: " + str(mid))    
        print("self.cmd.angular.z: " + str(self.cmd.angular.z))
        if speed > 1:
            speed = 1
        if speed < -1:
            speed = -1
        print("speed: " + str(speed)) 
        self.cmd.linear.x = speed   # TEMPORARILY SLOW SPEED

        self.drive_pub.publish(self.cmd)


rospy.init_node("ballDetector")
rate = rospy.Rate(60)
bD = InferBall()

while not rospy.is_shutdown():
    bD.infer_drive()
 #   rospy.spin()


