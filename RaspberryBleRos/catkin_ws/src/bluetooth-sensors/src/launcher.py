#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 28 14:44:05 2020

@author: agathe
"""

import kalman, ble_interface

bluno, ch = connectToBLE()
try:
    while True:
        if bluno.waitForNotifications(3.0): # calls handleNotification()
             continue
        print("Waiting...")
        dataSensors,Γsensors,u,A,measurements,C,dataSensorsEstimed,Γα,Γβ = initKalmanFilter(7.13,2.36)
        dataSensors, measurements, dataSensorsEstimed, Γsensors = updateKalmanFilter(dataSensors,measurements,dataSensorsEstimed,0,Γsensors,Γα,Γβ,A,C)
finally:
    bluno.disconnect()