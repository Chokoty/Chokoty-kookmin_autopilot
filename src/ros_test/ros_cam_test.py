#! /usr/bin/env python

from email.mime import image
import rospy

import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge,CvBridgeError

class camera_sim():

    def __init__(self):
        rospy.init_node('image_to_receiver', anonymous=False)
        self.pubcam = rospy.Publisher("/pub_image", Image, queue_size=10)
        self.subcam = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.bridge = CvBridge()
        rospy.on_shutdown(self.cam_shutdown)
        rospy.spin()

    def callback(self, data):
        # simulation cam -> cv2
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("converting error")
            print(e)
        height, width, channel = cv_image.shape
        #print(cv_image.shape)

        # cv2_bgr -> convert color
        cvt_img=cv2.cvtColor(cv_image,cv2.COLOR_RGB2HLS)
        h, s, v = cv2.split(255-cvt_img)
        lower = (0,0,0) 
        upper = (255,255,255)
        filtered_img = cv2.inRange(cvt_img, lower, upper)
        img_result = cv2.bitwise_and(cv_image, cv_image, mask = filtered_img)

        # ROI(region of Interest) 
        region = np.array([[(0, height), (0, height*60/100), (width/2, height*40/100),(width, height*60/100), (width, height)]], dtype = np.int32)
        zero_img = np.zeros_like(cv_image)
        region_mask=cv2.fillPoly(zero_img, region, (255,255,255))
        
        # Covered ROI Mask
        masked_img = cv2.bitwise_and(img_result, region_mask)
        
        # Warped Image
        src=np.float32([[0,height],[width,height], [width*25/100, height*70/100],[width*75/100, height*70/100]])
        dst=np.float32([[width*15/100,height],[width*85/100,height],[width*15/100,0],[width*85/100,0]])
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        warped_img = cv2.warpPerspective(masked_img, M, (width, height))

        # Calculate Curved
        
        try :
            self.pubcam.publish(self.bridge.cv2_to_imgmsg(warped_img,"rgb8"))
        except CvBridgeError as e:
            print("publish error")
            print(e)

    def cam_shutdown(self):
        print("I'm dead!")

if __name__=="__main__":

    cs = camera_sim()