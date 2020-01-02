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
dir = os.environ['HOME'] + '/work/jetbot/infer'
sys.path.insert(0, dir)
import trt_inference as infer

class InferBall:

    def __init__(self):
        self.imagePush = None
        self.cmd = Twist()
        self.cam = cv.VideoCapture(1)
        if self.cam.isOpened() is False:
            print("Cam open failed")
            sys.exit(1)
        self.cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        self.ret = None
        self.frame = None
        self.drive_pub = rospy.Publisher("/jetsoncar_teleop_joystick/cmd_vel", Twist, queue_size = 1)
        infer.setup()
        self.near = 80.
        self.far = 30.
        self.dist = 80.
        self.stop = False

    def infer_drive(self):
        #print("running")
        
        self.ret, self.frame = self.cam.read()
        if (self.frame) is None:
            time.sleep(0.5)
            print("sleeping")
            return None
        
        # TEMPORARILY GOT RID OF ALL PRINTING SO THAT I COULD PRINT IN THE OTHER FILE WITH CLARITY
        
        # Detect the bounding boxes around the tennis ball and gestures
        boxes, classes, scores, amount = infer.inferenceBox(self.frame)

        for i in range(0,amount):
            if classes[i] == 1: # If a tennis ball is detected
                if scores[i] > 0.75:
                    mid = (boxes[i][3] + boxes[i][1])/2
                    area = (boxes[i][2] - boxes[i][0]) * (boxes[i][3] - boxes[i][1]) * 10000      # Scaled area of the tennis ball box for estimation of distance
                    # Set the speed and angle values according to the bounding box     
                    self.cmd.angular.z = ((mid*2) - 1) * -1   # Set the angle aspect of the ROS variable
                    speed = ((area/self.dist) - 1) * - 1  
            elif classes[i] == 2:  # If the "come closer" gesture is detected
                if scores[i] > 0.75:
                    self.dist = self.near
            elif classes[i] == 3:  # If the "back up" gesture is detected
                if scores[i] > 0.75:
                    self.dist = self.far
            elif classes[i] == 4:  # If the "stop" gesture is detected
                if scores[i] > 0.75:
                    self.stop = True
            elif classes[i] == 5:  # If the "go" gesture is detected
                    self.stop = False
            else:
                self.cmd.angular.z = 0
                speed = 0                        
        # Keep speed between -1 and 1
        if speed > 1:
            speed = 1
        if speed < -1:
            speed = -1
        
        if self.stop is True:
            self.cmd.angular.z = 0
            speed = 0
        # Set the speed aspect of the ROS variable
        self.cmd.linear.x = speed 

        # Publish the ROS variable to the topic that the Arduino is subscribed to
        self.drive_pub.publish(self.cmd)


rospy.init_node("ballDetector")
rate = rospy.Rate(60)
bD = InferBall()

while not rospy.is_shutdown():
    bD.infer_drive()
 #   rospy.spin()


