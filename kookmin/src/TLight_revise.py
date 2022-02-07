#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time


class TLight() :

    def __init__(self) :
        self.tlight_sub = rospy.Subscriber("/GetTrafficLightStatus", CompressedImage, self.tlight_callback)
        self.status = 0
        self.tlight_str = "Nothing"
        self.stop_time = 4
        self.time = time.time()
        self.time_flag = True
        # 평상시 status 0
        # 노란불 혹은 빨간불 일시 status 1
        # status 1 상태에서 n초 지난 후 status 2
        # status 2 상태에서 좌회전 파란불 혹은 파란불 시 status 0

    
    def tlight_callback(self, data):
        self.tlight_num = data


    def detect(self):
        if self.time_flag and self.tlight_num != 0:
            self.time = time.time()
            self.time_flag = False
        if ( self.status == 0 ) and ( self.tlight_str == "Y" or self.tlight_str == "R" ) :
            self.status = 1
        elif ( self.status == 1 ) and ( self.time + self.stop_time < time.time() ) :
            self.status = 2
        elif ( self.status == 2 ) and ( self.tlight_str == "SG" or self.tlight_str == "R_with_GLeft" ) :
            self.status = 0
            self.time_flag = True
            self.tlight_num = 0
