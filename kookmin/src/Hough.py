#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge,CvBridgeError


class PID():

    def __init__(self, kp, ki, kd):

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):

        self.d_error = cte - self.p_error	# (현재 시간 error) - (이전 시간 error)
        self.p_error = cte
        if abs(self.i_error + cte) < 2000:	# 시간이 지날수록 계속 누적되면 i_error가 p나 d에 비해 너무 커지기에 상한선 설정
            self.i_error += cte			# ki = 0.0005인 경우, 0.0005 x 2000 = 1. 적분 부분이 1도 이상 영향 주지 않게끔 설정

        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error


class Hough() :

    def __init__(self) :
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.bridge = CvBridge()
        self.hough_speed = 0
        self.hough_angle = 0

        self.image = np.empty(shape=[0])
        self.roi = np.empty(shape=[0])
        self.Width = 640
        self.Height = 480
        self.channel = 3

        # Tuning Parameters
        self.Offset = 305               # Used in process_image (roi)
        self.Gap = 40                   # Used in process_image (roi)
        self.canny_low_thres = 60       # Used in process_image (canny)
        self.canny_high_thres = 70      # Used in process_image (canny)
        self.low_slope_thres = 0        # Used in divide_left_right
        self.high_slope_thres = 10      # Used in divide_left_right


    # 카메라로부터 Image형 토픽이 오면 전처리 시킬 수 있는 이미지로 변환 작업
    def img_callback(self, data):
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("converting error")
            print(e)
        # height, width, channel = self.image.shape
        # printf(height, width, channel)


    def drive(self):
        pid_c = PID(0.5, 0.0, 0.0)			# 곡선(curve)에서의 PID
        pid_s = PID(0.25, 0.0005, 0.3)		# 직선(straight)에서의 PID

        while not self.image.size == (self.Width*self.Height*self.channel):
            return
        # cv2.imshow("Image", self.image)   # 전처리 전 입력이미지 출력

        self.image = cv2.resize(self.image, (self.Width, self.Height))
        lpos, rpos = self.process_image()

        center = (lpos + rpos) / 2
        error = self.Width/2 - center

        if lpos == 0.0 or rpos > 630.0:			# 곡선(한쪽 차선을 검출 못하거나 오른쪽 차선이 너무 오른쪽에 있는 경우 곡선으로 가정)
            self.hough_angle = pid_c.pid_control(error)		# angle = error/2
            self.hough_speed = 22
        else:						            # 직선
            self.hough_angle = pid_s.pid_control(error)
            self.hough_speed = 28


    def process_image(self):
        # gray
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        # roi
        self.roi = gray[self.Offset : self.Offset + self.Gap, 0 : self.Width]
        # blur
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(self.roi,(kernel_size, kernel_size), 0)
        # canny edge
        edge_img = cv2.Canny(np.uint8(blur_gray), self.canny_low_thres, self.canny_high_thres)
        # HoughLinesP
        all_lines = cv2.HoughLinesP(edge_img,1,math.pi/180,30,30,10)

        # divide left, right lines
        if all_lines is None:
            return 0, 640
        left_lines, right_lines = self.divide_left_right(all_lines)

        # get center of lines
        lpos = self.get_line_pos(left_lines, left=True)
        rpos = self.get_line_pos(right_lines, right=True)

        # draw lines
        self.draw_lines(left_lines)
        self.draw_lines(right_lines)
        self.image = cv2.line(self.image, (230, 235), (410, 235), (255,255,255), 2)
                                    
        # draw rectangle
        self.draw_rectangle(frame, lpos, rpos, offset=self.Offset)

        # show image
        cv2.imshow('after image processing', self.image)

        return lpos, rpos


    # 검출한 모든 선들을 왼쪽 차선과 오른쪽 차선으로 분리
    def divide_left_right(self, lines):
        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                slope = 0
            else:
                slope = float(y2-y1) / float(x2-x1)
            
            if abs(slope) > self.low_slope_thres and abs(slope) < self.high_slope_thres:
                if (slope < 0) and (x2 < Width/2 - 90):
                    left_lines.append([line[0].tolist()])
                elif (slope > 0) and (x1 > Width/2 + 90):
                    right_lines.append([line[0].tolist()])

        return left_lines, right_lines


    def get_line_pos(self, lines, left=False, right=False):
        m, b = self.get_line_params(lines)

        if m == 0 and b == 0:
            if left:
                pos = 0		    # 왼쪽 차선이 검출되지 않으면 lpos = 0
            if right:
                pos = Width		# 오른쪽 차선이 검출되지 않으면 rpos = 640
        else:
            y = self.Gap / 2
            pos = (y - b) / m

            b += self.Offset
            x1 = (self.Height - b) / float(m)
            x2 = ((self.Height/2) - b) / float(m)

            cv2.line(self.image, (int(x1), self.Height), (int(x2), (self.Height/2)), (255, 0,0), 3)

        return int(pos)


    # 선들의 x, y좌표들의 평균으로 기울기와 y절편 구하는 함수
    def get_line_params(self, lines):
        # sum of x, y, m
        x_sum = 0.0
        y_sum = 0.0
        m_sum = 0.0

        size = len(lines)
        if size == 0:
            return 0, 0

        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        # line 하나당 x1, x2로 x좌표가 2개씩이므로 size*2로 나눈다.
        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m = m_sum / size
        b = y_avg - m * x_avg

        return m, b


    def draw_lines(self, lines):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            self.image = cv2.line(self.image, (x1, y1+self.Offset), (x2, y2+self.Offset), color, 2)


    def draw_rectangle(self, lpos, rpos, offset=0):
        center = (lpos + rpos) / 2

        # 왼쪽 차선
        cv2.rectangle(self.image, (lpos - 5, 15 + offset),
                        (lpos + 5, 25 + offset),
                        (0, 255, 0), 2)
        # 오른쪽 차선
        cv2.rectangle(self.image, (rpos - 5, 15 + offset),
                        (rpos + 5, 25 + offset),
                        (0, 255, 0), 2)
        # 두 차선의 중앙 지점
        cv2.rectangle(self.image, (center-5, 15 + offset),
                        (center+5, 25 + offset),
                        (0, 255, 0), 2)    
        # 화면의 중앙
        cv2.rectangle(self.image, (315, 15 + offset),
                        (325, 25 + offset),
                        (0, 0, 255), 2)
