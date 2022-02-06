#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
import sys

from sensor_msgs.msg import LaserScan, PointCloud # PointCloud Velodyne message type
from geometry_msgs.msg import Point32 # velodyne 16 channels
from std_msgs.msg import Float64

class rpScanReceiver():
    
    def __init__(self):
        rospy.init_node('scan_py_receiver', anonymous=False)
        self.sublidar = rospy.Subscriber("/lidar2D", LaserScan, self.callback)
        self.publidar = rospy.Publisher("/pub_scan", PointCloud, queue_size=10)

        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('sim_test')
        full_path = pkg_path+'/scripts/'+'lidar_data.txt'
        self.f = open(full_path, 'a')

        rospy.on_shutdown(self.rpscan_shutdown)
        rospy.spin()

    def callback(self, data):
        min_angle = data.angle_min
        max_angle = data.angle_max
        min_range = data.range_min
        max_range = data.range_max
        
        angle_increment = data.angle_increment
        time_increment = data.time_increment
        lidar_header = data.header
        PC_data = PointCloud()
        PC_data.header=data.header
        #print("data len : {}".format(len(data.ranges)))
        for i in range(len(data.ranges)):
            x, y = self.calc_axis_xy(min_angle+angle_increment*i,data.ranges[i], min_range,max_range)
            if [x,y] !=[0,0]:
                PC_data.points.append(Point32(x,y,1))
        self.f.write(str(x)+'\n')
        self.publidar.publish(PC_data)

    def calc_axis_xy(self, _theta, _distance, _min_range, _max_range):
        if _min_range <= _distance <= _max_range:
            x = np.cos(_theta) * _distance
            y = np.sin(_theta) * _distance
            return (x,y)
        else :
            return (0,0)
    
    def rpscan_shutdown(self):
        print("I'm dead!")
        self.f.close()

if __name__=="__main__":

    rp = rpScanReceiver()
