#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
import sys

from sensor_msgs.msg import Imu

class imuReceiver():
    
    def __init__(self):
        rospy.init_node('imu_py_receiver', anonymous=False)
        self.subimu = rospy.Subscriber("/imu", Imu, self.callback)
        self.pubimu = rospy.Publisher("/pub_imu", Imu, queue_size=10)

        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('sim_test')
        full_path = pkg_path+'/scripts/'+'imu_data.txt'
        self.f = open(full_path, 'a')

        rospy.on_shutdown(self.imu_shutdown)
        rospy.spin()

    def callback(self, data):
        ori_x = data.orientation.x
        ori_y = data.orientation.y
        ori_z = data.orientation.z
        ang_x = data.angular_velocity.x
        ang_y = data.angular_velocity.y
        ang_z = data.angular_velocity.z
        lin_x = data.linear_acceleration.x
        lin_y = data.linear_acceleration.y
        lin_z = data.linear_acceleration.z
        ori_cov=data.orientation_covariance
        ang_vel_cov=data.angular_velocity_covariance
        lin_acc_cov=data.linear_acceleration_covariance
        imu_header = data.header
        custom_imu = Imu()
        custom_imu.header=data.header
        #print("data len : {}".format(len(data.ranges)))

        self.f.write(str(ori_x)+'\n')
        self.pubimu.publish(custom_imu)
    
    def imu_shutdown(self):
        print("I'm dead!")
        self.f.close()

if __name__=="__main__":

    ir = imuReceiver()
