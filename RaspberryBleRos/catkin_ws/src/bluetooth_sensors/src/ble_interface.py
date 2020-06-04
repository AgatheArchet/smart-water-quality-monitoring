#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  3 15:16:28 2020

@author: agathe
"""

import rospy
from bluetooth_sensors.msg import SensorsData
import signal
import sys
from bluepy.btle import Peripheral, DefaultDelegate

class SensorsDelegate(DefaultDelegate):
    """
    Deleguate object from bluepy library to manage notifications from the Arduino Bluno through BLE.
    """
    def __init__(self):
        DefaultDelegate.__init__(self)
        # ... initialise here

    def handleNotification(self, cHandle, data):
        """
        Sorts data transmitted by Arduino Bluno through BLE.
        """
        if (cHandle==37):
#            print(data)
            if (len(data)>6):
                message = data.decode()
                if(message[0]=="M"):
                    try:
                        month, day, year, hour, minute, second = message.replace("M"," ").replace("d"," ").replace("y"," ").replace("h"," ").replace("m"," ").replace("s"," ").split()
                        #print(month+"/"+day+"/"+year+"  "+hour+":"+minute+":"+second)
                        msg.date = "ok"
			print("date")
                    except ValueError:
                        pass
                elif(message[0:2]=="pH"):
                    try:
                        phValue, tpValue, ecValue = message.replace("pH"," ").replace("tp"," ").replace("EC"," ").split()
                        #print("pH : "+phValue+" Temp : "+tpValue+"°C  Conductivity : "+ecValue+" mS/cm")
                        msg.ph, msg.temperature,msg.conductivity = float(phValue), float(tpValue),float(ecValue)
                        print("ph")
                    except ValueError:
                        pass
                elif(message[0:2]=="Do"):
                    try:
                        doValue, orValue = message.replace("Do"," ").replace("Or"," ").split()
                        #print ("Dissolved Oxygen : "+doValue+" mg/L  Redox potential : "+orValue+" mV")
                        msg.dissolved_oxygen, msg.redox_potential = float(doValue), float(orValue)
			print("or")
                    except ValueError:
                        pass
            else:
                pass

def signal_handler(sig, frame):
    print('  You pressed Ctrl+C! Bluno disconnected')
    if ('bluno' in globals()):
    	bluno.disconnect()
    msg.is_connected = False
    sys.exit(0)
    rospy.signal_shutdown('  Ctrl+C was pressed')

def connectToBLE():
    # connection to the device
    bluno = Peripheral("C8:DF:84:24:27:F6", "public")
    bluno.setDelegate(SensorsDelegate())

    svc = bluno.getServiceByUUID("dfb0")
    ch = svc.getCharacteristics("dfb1")[0]
    return (bluno, ch)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGQUIT, signal_handler);
signal.signal(signal.SIGTERM, signal_handler);
signal.signal(signal.SIGHUP, signal_handler);

pub = rospy.Publisher('raw_data_topic', SensorsData, queue_size=1)
rospy.init_node('ble_interface_node', anonymous=True, disable_signals=True)
rate = rospy.Rate(10)
msg = SensorsData()
msg.is_connected = False
msg.date = ""
msg.ph = -1
msg.conductivity = -1
msg.temperature = -1
msg.dissolved_oxygen = -1
msg.redox_potential = -1

print("Connecting to device...")
bluno, ch = connectToBLE()
print("\n Device connected")
msg.is_connected = True

try:
	while not rospy.is_shutdown():
                if bluno.waitForNotifications(5.0): # calls handleNotification()
                	rospy.loginfo(msg)
                	pub.publish(msg)
                	rate.sleep()

                print("Waiting...")
		#ch.write(str.encode("ok1"))

finally:
        bluno.disconnect()
	msg.is_connected = False
        pub.publish(msg)
        rate.sleep()
