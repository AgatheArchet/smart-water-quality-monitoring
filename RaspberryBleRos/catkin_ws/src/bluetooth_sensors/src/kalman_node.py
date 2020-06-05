#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 16:23:26 2020

@author: agathe
"""

import rospy
from bluetooth_sensors.msg import SensorsData

def callback(raw_data):
    print(raw_data)
    rospy.loginfo(filtered_data)
    pub.publish(filtered_data)

def initNode():
	rospy.init_node('kalman_filter_node')
	sub = rospy.Subscriber('raw_data_topic',SensorsData,callback)
	pub = rospy.Publisher('filtered_data_topic', SensorsData, queue_size=1)
	rate = rospy.Rate(10)
	filtered_data = SensorsData()
	filtered_data.is_connected = False
	filtered_data.date = ""
	filtered_data.ph = -1
	filtered_data.conductivity = -1
	filtered_data.temperature = -1
	filtered_data.dissolved_oxygen = -1
	filtered_data.redox_potential = -1
	return(sub,pub,rate,filtered_data)

sub,pub,rate,filtered_data = initNode()
rospy.spin() # to give some time for kalman fileterig

