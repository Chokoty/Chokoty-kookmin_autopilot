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

        rospy.init_node('Ego_py_speed_angle', anonymous=False)
        self.subEgo_speed = rospy.Subscriber("/sensors/core", VescStateStamped, self.speed_callback)
        self.subEgo_angle = rospy.Subscriber("/sensors/servo_position_command", Float64, self.angle_callback)
        self.subEgo_topic = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_callback)

        self.pubEgo_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=10)
        self.pubEgo_angle = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        self.cmd_speed = 0
        self.cmd_angle = 0.5
        #rospack=rospkg.RosPack()
        #pkg_path = rospack.get_path('sim_test')
        #full_path = pkg_path+'/scripts/'+'Ego_data.txt'
        #self.f = open(full_path, 'a')

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

        custom_Ego = EgoVehicleStatus()
        custom_Ego.header=data.header
        
        custom_Ego_position = Float64()

        #self.f.write(str(lat)+'\n')

    def speed_callback(self,data):
        ego_speed = data.state.speed
        if ego_speed < 2000 :
            cmd_speed = 10000
        else :
            cmd_speed = 10000
        Ego_speed_msg = Float64()

        Ego_speed_msg.data = cmd_speed
        self.pubEgo_speed.publish(Ego_speed_msg)

    def angle_callback(self,data):
        _angle = -10
        ego_angle = data.data
        now_angle=((ego_angle*2)-1)*19.5
        if now_angle < 10 :
            cmd_angle = round(_angle/19.5,5)
        Ego_angle_msg = Float64()
        Ego_angle_msg.data = cmd_angle
        print(cmd_angle)

        self.pubEgo_angle.publish(Ego_angle_msg)

    def Ego_shutdown(self):
        print("I'm dead!")
        custom_Ego_speed=0
        custom_Ego_position=0.5
        self.pubEgo_speed.publish(custom_Ego_speed)
        self.pubEgo_angle.publish(custom_Ego_position)


if __name__=="__main__":

    ER = EgoReceiver()
