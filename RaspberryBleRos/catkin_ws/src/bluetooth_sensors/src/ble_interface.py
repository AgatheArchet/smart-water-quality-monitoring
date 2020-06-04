#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 15:16:28 2020

@author: agathe
"""

import rospy
from bluetooth_sensors.msg import SensorsData


pub = rospy.Publisher('raw_data_topic', SensorsData)
rospy.init_node('ble_interface_node', anonymous=True)
rate = rospy.Rate(1) 
msg = SensorsData()
msg.is_connected = True
msg.date = ""
msg.ph = -1
msg.conductivity = -1 
msg.temperature = -1
msg.dissolved_oxygen = -1 
msg.redox_potential = -1

while not rospy.is_shutdown():
    #rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()
        
