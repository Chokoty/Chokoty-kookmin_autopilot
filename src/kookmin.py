#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
from numpy import angle
import rospy
import rospkg
import numpy as np
import sys
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
import time

#from hough_drive import Hough


class Kookmin(object):

    def __init__(self) :
        self.rate = rospy.Rate(10)
        self.pubEgo_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=10)
        self.pubEgo_angle = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)

        self.hough = Hough()
        self.stop = Stop()
        self.bump = Bumper()


        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('sim_test')
        full_path = pkg_path+'/scripts/'+'Ego_data.txt'
        self.f = open(full_path, 'a')

        rospy.on_shutdown(self.Ego_shutdown)
        rospy.spin()
        
    
    def control(self) :
        
        self.hough.drive()

        # Tracking(Stanley Method)가 속도와 조향각의 기본 베이스로 들어간다.
        self.msg.speed = self.stanley_follower.speed
        self.msg.angle = self.stanley_follower.angle
        
        # 방지턱이 없는 경우 bump_speed는 0으로 아무런 영향이 없고,
        # Imu센서를 통해 방지턱을 감지한 경우에 bump_speed가 15와 -25 등으로 바뀌며 속도에 영향을 주고, 조향각은 0이 된다.
        self.msg.speed += self.bump.bump_speed
        if self.bump.bump_speed != 0:
            self.msg.angle = 0
        
        # 정지선을 보면 2초 간 정지
        if self.stop.check == True :
            print("정지선")
            time.sleep(2)
            self.msg.speed = 20
            self.msg.angle = 0
            self.pub.publish(self.msg)
            self.stop.check = False


        self.pub.publish(self.msg)
        
        self.rate.sleep()
