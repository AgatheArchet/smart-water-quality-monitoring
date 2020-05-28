#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 28 14:44:05 2020

@author: agathe
"""

import binascii
import time
from bluepy.btle import Peripheral, UUID, DefaultDelegate
 
# tutorial for bluetooth settings : https://learn.adafruit.com/install-bluez-on-the-raspberry-pi/installation

class SensorsDelegate(DefaultDelegate):
    """
    Deleguate object from bluepy library to manage notifications from the ARduino Bluno through BLE.
    """
    def __init__(self):
        DefaultDelegate.__init__(self)
        # ... initialise here

    def handleNotification(self, cHandle, data):
        """
        Sorts data transmitted by Arduino Bluno through BLE.
        """
        if (cHandle==37):
            if (len(data)>6):
                msg = data.decode()
                if(msg[0]=="M"):
                    try:
                        month, day, year, hour, minute, second = msg.replace("M"," ").replace("d"," ").replace("y"," ").replace("h"," ").replace("m"," ").replace("s"," ").split()
                        print(month+"/"+day+"/"+year+"  "+hour+":"+minute+":"+second)
                    except ValueError:
                        pass
                elif(msg[0:2]=="pH"):
                    try:
                        phValue, tpValue, ecValue = msg.replace("pH"," ").replace("tp"," ").replace("EC"," ").split()
                        print("pH : "+phValue+" Temp : "+tpValue+"°C  Conductivity : "+ecValue+" mS/cm")
                    except ValueError:
                        pass
                elif(msg[0:2]=="Do"):
                    try:
                        doValue, orValue = msg.replace("Do"," ").replace("Or"," ").split()
                        print ("Dissolved Oxygen : "+doValue+" mg/L  Redox potential : "+orValue+" mV")
                    except ValueError:
                        pass
            else:
                pass

def connectToBLE():
    # connection to the device
    bluno = Peripheral("C8:DF:84:24:27:F6", "public")
    bluno.setDelegate(SensorsDelegate())
    
    svc = bluno.getServiceByUUID("dfb0")
    ch = svc.getCharacteristics("dfb1")[0]
    return (bluno, ch)
    

if __name__=='__main__':
    
    bluno, ch = connectToBLE()
    try:
        while True:
            #ch.write(str.encode("ok"))
            if bluno.waitForNotifications(1.0): # calls handleNotification()
                 continue
            print("Waiting...")
    finally:
        bluno.disconnect()