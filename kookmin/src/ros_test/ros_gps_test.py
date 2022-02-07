#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
import sys
from morai_msgs.msg import GPSMessage

class gpsReceiver():
    
    def __init__(self):
        rospy.init_node('gps_py_receiver', anonymous=False)
        self.subgps = rospy.Subscriber("/gps", GPSMessage, self.callback)
        self.pubgps = rospy.Publisher("/pub_gps", GPSMessage, queue_size=10)

        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('sim_test')
        full_path = pkg_path+'/scripts/'+'gps_data.txt'
        self.f = open(full_path, 'a')

        rospy.on_shutdown(self.gps_shutdown)
        rospy.spin()

    def callback(self, data):
        lat = data.latitude
        long = data.longitude
        alt = data.altitude
        east = data.eastOffset
        north = data.northOffset
        stat = data.status

        gps_header = data.header
        custom_gps = GPSMessage()
        custom_gps.header=data.header
        #print("data len : {}".format(len(data.ranges)))

        self.f.write(str(lat)+'\n')
        self.pubgps.publish(custom_gps)
    
    def gps_shutdown(self):
        print("I'm dead!")
        self.f.close()

if __name__=="__main__":

    gr = gpsReceiver()
