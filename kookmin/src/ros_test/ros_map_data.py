#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
import sys
#from math import round 
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from vesc_msgs.msg import VescStateStamped


class EgoReceiver():
    
    def __init__(self):

        rospy.init_node('Ego_py_map_data', anonymous=False)
        self.subEgo_topic = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_callback)
        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('sim_test')
        full_path = pkg_path+'/src/scripts/'+'map_data.txt'
        self.f = open(full_path, 'a')
        print(full_path)
        rospy.on_shutdown(self.Ego_shutdown)
        rospy.spin()

    def Ego_callback(self, data):
        Ego_header = data.header
        id=data.unique_id
        
        acc_x=data.acceleration.x
        acc_y=data.acceleration.y
        acc_z=data.acceleration.z
        
        pos_x=data.position.x
        pos_y=data.position.y
        pos_z=data.position.z

        vel_x=data.velocity.x
        vel_y=data.velocity.y
        vel_z=data.velocity.z
        
        heading=data.heading
        accel=data.accel
        brake=data.brake
        angle=data.wheel_angle

        self.f.write(str(pos_x)+'\t'+str(pos_y)+'\t'+str(pos_z)+'\n')

    def Ego_shutdown(self):
        print("I'm dead!")
        self.f.close()

if __name__=="__main__" :
    ER = EgoReceiver()
