#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 16:23:26 2020

@author: agathe
"""

import rospy
from bluetooth_sensors.msg import SensorsData

def callback(rawData):
    print(rawData)
    
rospy.init_node('kalman_filter_node')
sub = rospy.Subscriber('raw_data_topic',SensorsData,callback)

rospy.spin()
