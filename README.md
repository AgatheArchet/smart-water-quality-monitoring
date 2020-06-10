# smart-water-quality-monitoring
Project carried out as part of an internship on autonomous sailboats

## Part 1 : measurements on Arduino Bluno

The Arduino Bluno board must meet two objectives: obtaining and processing data providing by the different modules and sharing these data with a Raspberry Pi via Bluetooth Low Energy (BLE).   

A guide is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > [Measurements with Arduino Bluno](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki/Measurements-with-an-Arduino-Bluno)**.

## Part 2 : data sharing through BLE

The measurements and time can be sent from Arduino Bluno through Bluetooth Low Energy, and received by a Raspberry Pi 3 B+ (or other models with a BLE integrated module). The first device plays the *peripheral* role, the second one plays the *central* role.

A guide is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > [BLE communication between an Arduino Bluno and a Raspberry Pi](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki/BLE-communication-between-an-Arduino-Bluno-and-a-Raspberry-Pi)**.

## Part 3 : ROS achitecture on Raspberry Pi

Finally, once collected by the Raspberry Pi, the data must be correctly sorted and filtered before being sent to the user. A middleware architecture will be used (ROS melodic). 

A guide is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > A middleware achitecture on Raspberry Pi**.
