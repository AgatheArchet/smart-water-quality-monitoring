#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 09:51:43 2020

@author: agathe
"""
import rospy
from std_msgs.msg import String

def callback(msg):
    print(msg.data)
    
rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/phrases',String,callback)

rospy.spin()
    
    