#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  4 10:29:14 2020

@author: agathe
"""

import rospy
from bluetooth_sensors.msg import SensorsData

def callback(filtered_data):
    print(filtered_data)
    rospy.loginfo(filtered_data)
    
rospy.init_node('user_interface_node')
sub = rospy.Subscriber('filtered_data_topic',SensorsData,callback)


rospy.spin()
    
