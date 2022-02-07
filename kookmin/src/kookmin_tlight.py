#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2, random, math
from numpy import angle
import rospy, rospkg
import numpy as np
from std_msgs.msg import Float64
from TLight_revise import TLight


class Kookmin(object):

    def __init__(self) :
        self.rate = rospy.Rate(10)
        self.pubEgo_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=10)
        self.pubEgo_angle = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        self.tlight = TLight()


    def control(self) :
        
        self.tlight.detect()    # 신호등 신호로 status 갱신

        if self.tlight.status < 2:
            custom_Ego_speed = 100.0
            custom_Ego_position = 0
        elif self.tlight.status == 2:
            custom_Ego_speed = 0
            custom_Ego_position = 0

        self.pubEgo_speed.publish(custom_Ego_speed)
        self.pubEgo_angle.publish(custom_Ego_position)
        
        self.rate.sleep()
