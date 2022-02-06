#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2, random, math
from numpy import angle
import rospy, rospkg
import numpy as np
import sys
from std_msgs.msg import Float64
import time
from wecar_plan import wecar_planner


class Kookmin(object):

    def __init__(self) :
        self.rate = rospy.Rate(10)
        self.pubEgo_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=10)
        self.pubEgo_angle = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        self.wplanner = wecar_planner()


    def control(self) :
        self.wplanner.wecar_running()
        custom_Ego_speed = self.wplanner.motor_msg
        custom_Ego_position = self.wplanner.servo_msg
        self.pubEgo_speed.publish(custom_Ego_speed)
        self.pubEgo_angle.publish(custom_Ego_position)
        
        self.rate.sleep()
