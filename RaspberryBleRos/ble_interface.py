import binascii
import struct
import time
from bluepy.btle import Peripheral, UUID, DefaultDelegate
 
class SensorsDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        # ... initialise here

    def handleNotification(self, cHandle, data):
        if (cHandle==37):
            msg = data.decode()
            if(msg[0]=="M"):
                cursor = 0
                if(msg[cursor+2]=='d'):
                    month = msg[cursor+1]
                else:
                    month = msg[cursor+1:cursor+3]
                cursor += len(month)+1
                if(msg[cursor+2]=='y'):
                    day = msg[cursor+1]
                else:
                    day = msg[cursor+1:cursor+3]
                cursor += len(day)+1
                year = msg[cursor+1:cursor+5]
                cursor += 5
                if(msg[cursor+2]=='m'):
                    hour = msg[cursor+1]
                else:
                    hour = msg[cursor+1:cursor+3]
                cursor += len(hour)+1
                if(msg[cursor+2]=='s'):
                    minute = msg[cursor+1]
                else:
                    minute = msg[cursor+1:cursor+3]
                cursor += len(minute)+1
                second = msg[cursor+1:]
                print(month+"/"+day+"/"+year+"  "+hour+":"+minute+":"+second)
            elif(msg[0:2]=="pH"):
                phValue = msg[2:6]
                tpValue = msg[8:12]
                ecValue = msg[14:18]
                print("pH : "+phValue+" Temp : "+tpValue+"°C  Conudctivity : "+ecValue+" mS/cm")
            elif(msg[0:2]=="Do"):
                doValue = msg[2:6]
                orValue = msg[8:12]
                print ("Dissolved Oxygen : "+doValue+"  Redox potential : "+orValue)
        

# connection to the device
bluno = Peripheral("C8:DF:84:24:27:F6", "public")
bluno.setDelegate(SensorsDelegate())

svc = bluno.getServiceByUUID("dfb0")
ch = svc.getCharacteristics("dfb1")[0]

try:
    while True:
        ch.write(str.encode("ok"))
        if bluno.waitForNotifications(1.0): # calls handleNotification()
             continue
        print("Waiting...")
finally:
    bluno.disconnect()