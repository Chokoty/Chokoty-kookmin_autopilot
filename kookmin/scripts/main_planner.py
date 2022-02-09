#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import sys
from kookmin_planner import Kookmin

rospy.init_node('main')

arg = rospy.myargv(argv=sys.argv)
path_name = arg[1]
kookmin = Kookmin(path_name)

while not rospy.is_shutdown():
    kookmin.control()
