import binascii
import struct
import time
from bluepy.btle import UUID, Peripheral
 
temp_uuid = UUID("dfb1")
 
p = Peripheral("C8:DF:84:24:27:F6", "public")
 
try:
    ch = p.getCharacteristics(uuid=temp_uuid)[0]
    print(ch)
    if (ch.supportsRead()):
        while 1:      
            print(ch.getHandle())
#            print(ch.read())
#            val = binascii.b2a_hex(ch.read())
#            val = binascii.unhexlify(val)
#            val = struct.unpack('f', val)[0]
#            print(str(val)) # + " deg C"
            time.sleep(1)
 
finally:
    p.disconnect()