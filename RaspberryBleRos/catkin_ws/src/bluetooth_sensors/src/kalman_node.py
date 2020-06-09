#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 16:23:26 2020

@author: agathe
"""

import rospy
from bluetooth_sensors.msg import SensorsData
import numpy as np
import signal, sys

def signalHandler(sig, frame):
    """
    Catches kill signals to terminate the node and disconnect the Arduino Bluno.
    """
    print('  You pressed Ctrl+C! Kalman proprely terminated')
    if ('f' in globals()):

        print('  file closed')
        rospy.signal_shutdown('  Ctrl+C was pressed')
        f.close()
        sys.exit(0)
        
def initSignalHandler():
	"""
	Details all potential kill signals.
	"""
	# signal handler for a proper disconnection with ROS
	signal.signal(signal.SIGINT, signalHandler)
	signal.signal(signal.SIGQUIT, signalHandler);
	signal.signal(signal.SIGTERM, signalHandler);
	signal.signal(signal.SIGHUP, signalHandler);
    
def kalman(x0,G0,u,y,Ga,Gb,A,C):
    """
    Corrects and predicts the right values of state vector x using 
    the Kalman filter.
    """
    # correction
    S = np.dot(np.dot(C,G0),C.T) + Gb        
    K = np.dot(np.dot(G0,C.T),np.linalg.inv(S))           
    ytilde = y - np.dot(C,x0)        
    Gup = np.dot((np.eye(len(x0))-np.dot(K,C)),G0) 
    xup = x0 + np.dot(K,ytilde)
    # prediction
    G1 = np.dot(np.dot(A,Gup),A.T) + Ga
    x1 = np.dot(A,xup) + u 
    return(x1,G1) 
    
def update(xnew):
    """
    Updates values of state vector x and observation vector y with
    data collected from simulation or BLE.
    """
    # uncertainties on the model modelised with gaussian noises
    x = xnew
    y = np.array([[xnew[0,0,]],[xnew[1,0]]])
    return(x,y)
    
def updateKalmanFilter(xnew,xhat,u,Gx,Ga,Gb,A,C):
    """
    Collects new values measured and applies them to the Kalman filter.
    x : true values, y : measurements, xhat : kalman-Estimated values  
    """
    x,y = update(xnew)
    xhat, Gx = kalman(xhat,Gx,u,y,Ga,Gb,A,C)
    return(xhat,Gx)
    
def initKalmanFilter(firstPhValue,firstEcValue):
    """
    Initializes all variables to use the Kalman filter for pH and conductivity 
    measurements.
    """
    # model :  x = A*x + u 
    x = np.array([[firstPhValue],[firstEcValue]]) # collected from the Arduino
    Gx = np.diag([0.2,0.2]) # estimated error on initial state
    u = 0 # no command
    A = np.eye(2)
    
    # measurements : y = C*x 
    y = np.array([[x[0,0]],[x[1,0]]])
    C = np.eye(2)
    
    #  estimation : x_ = A*x + u + a / y_ = C*x_ + b
    xhat = np.array([[x[0,0]],[x[1,0]]])
    Ga = np.diag([0.5,0.5]) # estimated error on model
    Gb = np.diag([0.2,0.2]) # estimated error on each mesaurement
    
    return(x,Gx,u,A,y,C,xhat,Ga,Gb)
    
def initNode():
    """
    Initializes the node,the associated publisher and subscriber.
    """
    rospy.init_node('kalman_filter_node',anonymous=True, disable_signals=True)
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
    
def callback(raw_data):
    """
    Notifications perceived by the ROS subscriber.
    """
    global sub,pub,rate,filtered_data, initialized, f
    global dataSensors,Gsensors,u,A,measurements,C,dataSensorsEstimated,Ga,Gb
    if ((raw_data.ph!=1) and (raw_data.conductivity!=1)):
        if not(initialized):
            print("Kalman filter initialization")
            dataSensors,Gsensors,u,A,measurements,C,dataSensorsEstimated,Ga,Gb = initKalmanFilter(raw_data.ph, raw_data.conductivity)
            initialized = True
        # Kalman filter 
        dataSensors = np.array([[raw_data.ph],[raw_data.conductivity]]) 
        dataSensorsEstimated, Gsensors = updateKalmanFilter(dataSensors,dataSensorsEstimated,u,Gsensors,Ga,Gb,A,C)
        if not(f.closed):
            f.writelines("%.3f;%.3f;%.3f;%.3f\n" % (raw_data.ph,dataSensorsEstimated[0,0],raw_data.conductivity,dataSensorsEstimated[1,0]))
            #print("writting to file...")
        # update filtered_data message
        filtered_data = raw_data
        filtered_data.ph, filtered_data.conductivity = dataSensorsEstimated[0,0], dataSensorsEstimated[1,0]
    else:
        filtered_data = raw_data
    #rospy.loginfo(filtered_data)
    try:
        pub.publish(filtered_data) #once is filtered
    except:
        pass

initSignalHandler()
sub,pub,rate,filtered_data = initNode()
initialized = False
dataSensors,Gsensors,u,A,measurements,C,dataSensorsEstimated,Ga,Gb = None,None,None,None,None,None,None,None,None
f = open("/home/agathe/Documents/A2/Stage/Projet/smart-water-quality-monitoring/RaspberryBleRos/catkin_ws/src/bluetooth_sensors/src/kalman_graph.txt","w+")

rospy.spin() # subscriber callback






