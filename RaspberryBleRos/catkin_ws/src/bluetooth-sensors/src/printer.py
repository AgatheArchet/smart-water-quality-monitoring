#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  2 14:48:40 2020

@author: agathe
"""

import rospy
from std_msgs.msg import String 
rospy.init_node('printer_node')
pub = rospy.Publisher('/phrases', String, queue_size=18)

rate = rospy.Rate(10)
msg_str = String()
msg_str = "Hellow world"

while not rospy.is_shutdown():
    pub.publish(msg_str)
    rate.sleep()
    
