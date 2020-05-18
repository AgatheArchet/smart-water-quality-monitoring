# smart-water-quality-monitoring
Project carried out as part of an internship on autonomous sailboats

## Common debbuging

If uploading code through USB cable seems impossible and the shell prints "*Error: avrdude: stk500_getsync()*", it may be caused by a wrong AT configuration. There is a need to reinstall the firmware from Windows to get back the default settings :

1. Download the DFRobot blunoFWDownloader on Windows at :   
https://www.dfrobot.com/image/data/DFR0267/Bluno%20Firmware%20Downloader/BlunoFWDownloaderV1.0%20windows.zip
2. Download last firmware version at https://github.com/DFRobot/BLE_firmware_V1.9, and decompress the file to obtain the SBL_BlunoV*.bin file.
3. While pressing *BOOT* button down on Bluno, connect the USB cable. *LINK* and *PAIR* leds are alternately flashing at this step.
4. Open the blunoFWDownloader application, select the port, select the firmware file previously download, at 115200 baurate. 
5. Launch the reinstallation
6. Open the Arduino application, then the Serial Monitor.
7. Enter "+++" on the command bar, with options : "No line ending" and "115200  baud". CLick on "send" to enter the AT mode.
8. Now set the options on "Both NL & CR". Enter "AT+SETTING=DEFAULT" in the command bar, click on ""send".
9. Do it again until a "OK" appears.
10. Enter "AT+EXIT" in the command bar, click on ""send". 




