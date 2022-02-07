#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2, random, math
from numpy import angle
import rospy, rospkg
import numpy as np
import sys
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import time
# from morai_msgs.msg import EgoVehicleStatus

#from hough_drive import Hough


class Kookmin(object):

    def __init__(self) :
        self.rate = rospy.Rate(10)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
       
        self.motor_msg=Float64()
        self.servo_msg=Float64()

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.bridge = CvBridge()
        self.image = np.empty(shape=[0])
        self.roi = np.empty(shape=[0])
        self.Width = 640
        self.Height = 480
        self.channel = 3
#        self.img = cv2.imread('/home/cooper/sim_ws/src/kookmin/src/girl.png', cv2.IMREAD_COLOR)
        print("init complete!!!!!!!!!!!!!!!!!!")

    def img_callback(self, data):
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("converting error")
            print(e)
        print("image show")
        cv2.imshow("Image", self.image)   # 입력이미지 출력
        cv2.waitKey(1)
        print("image show done")


    def control(self) :
        
        print("Control!!!")
        custom_Ego_speed = 1000.0
        custom_Ego_angle = 0.7 # -19.5 0 19.5 <-> 0 0.5 1.0 
        self.motor_msg = custom_Ego_speed
        self.servo_msg = custom_Ego_angle
        self.motor_pub.publish(self.motor_msg)
        self.servo_pub.publish(self.servo_msg)
        
        self.rate.sleep()
