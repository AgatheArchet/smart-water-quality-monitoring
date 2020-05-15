# smart-water-quality-monitoring
Project carried out as part of an internship on autonomous sailboats

## Part 1 : measurements on Arduino Bluno

The Arduino Bluno board must meet two objectives : obtaining and processing datas providing by the differents modules, and sharing these datas with a Raspberry Pi via Bluetooth Low Energy (BLE). The first part here describes how to achieve the first objective.

#### 1. Prepare the Arduino Bluno

Material needed :
* 1 Arduino Bluno (DFRobot Bluno = Uno + BLE integrated module)
* 1 IO Expansion Shield (DFRobot V7.1)
* 1 Real Time Clock circuit board (RTC) and its I2C 4-Pin sensor cable
* 1 Micro-SD module
* 1 Micro-SD card
* 1 Analog Signal isolator and its analog cable
* 1 pH probe and its circuit board
* 1 EC probe and its circuit board
* 2 Analog sensor cables

As a first step, the project focuses on two sensors : a pH and EC probes. An analog signal isolator is added to avoid signal interference between the two sensors.

Connecting all elements :

a. Add the IO expension shield on the Arduino bluno.  
b. Connect the pH module : plug the white end of the analog ("blue-red-black") cable to the pH data transfer board, and its black end to port A2 of the Bluno.The pH circuit board is now connected.  
c. Connect the EC module : plug the white end of the analog ("blue-red-black") cable to the analog signal isolator module (MCU side), and its black end to port A1.  
d. Plug one end of the "orange-red-black" cable to the EC circuit board, other end to the IN terminal of the isolation module (sendor side). The EC circuit board is now connected.  
e. Connect the Real Time Clock (RTC) module: Plug the white end of the I2C 4-pins ("blue-green-red-black") cable to RTC, and its black end to the blue I2C interface.  
f. Connect the micro-SD card module to the blue SD card slot. When inserted, the SD card should be pointing outwards the Arduino.  

TO DO : add photos for each step, talk about precautions to have with probes, how to connect the arduino and which steps to obtain the measurements/the calibration.







