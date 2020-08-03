# smart-water-quality-monitoring
Project carried out as part of an internship on autonomous sailboats.  
* First goal: the boat must be equipped with various sensors to analyze the quality of water in a certain area. Part 1 explains how the sensors are connected and their data exploited, part 2 details the communication strategy used between the boat and the sensors kit, and part 3 deals with final data processing on the boat.
* Second goal: the boat has to make measurements at various points on a map, with constraints linked to the environment. Its route must be planned to be the shortest and the quickest as possible.


## Part 1 : measurements on Arduino Bluno

The Arduino Bluno board must meet two objectives: obtaining and processing data providing by the different modules and sharing these data with a Raspberry Pi via Bluetooth Low Energy (BLE).   

A guide is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > [Measurements with Arduino Bluno](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki/Measurements-with-an-Arduino-Bluno)**.

## Part 2 : data sharing through BLE

The measurements and time can be sent from Arduino Bluno through Bluetooth Low Energy, and received by a Raspberry Pi 3 B+ (or other models with a BLE integrated module). The first device plays the *peripheral* role, the second one plays the *central* role.

A guide is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > [BLE communication between an Arduino Bluno and a Raspberry Pi](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki/BLE-communication-between-an-Arduino-Bluno-and-a-Raspberry-Pi)**.

## Part 3 : ROS achitecture on Raspberry Pi

Finally, once collected by the Raspberry Pi, the data must be correctly sorted and filtered before being sent to the user. A middleware architecture will be used (ROS melodic). 

A guide is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > [A middleware achitecture on Raspberry Pi](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki/A-middleware-architecture-on-Raspberry-Pi)**.

## Part 4 : preparation of the boat and the model for path planning

To facilitate path planning, the problem modelling should take into account all constraints associated with the sailboat's properties, the wind's speed and direction as well as the measurement area's borders and obstacles.

A description is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > [Modelling for the Travelling Salesman Problem](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki/Modelling--for-the-Travelling-Salesman-Problem)**.

## Part 5 : autonomous sailboat on TSP (static algorithms)

Now that the sailboat and the measurement zone are defined and modeled, various algorithms can be implemented to solve this specific Travelling Salesman Problem. To make the boat autonomous, various maps and measurement patterns will be used. 

A description is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > 

## Part 6 : obstacles avoidance strategies

The majority of coastal areas have irregular borders and obstacles. Then, the autonomous boat must integrate obtacle avoidance strategies when planning its path. 

A description is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > 

## Part 7 : intelligent sailboat on TSP (Q-Learning algorithm)

All paths calculated with previous algorithms offer solutions, but not necessarily the optimal ones. Further approaches such as Q-Learning and Deep Q-Learning algorithms could provide more interesting long-term strategies. 

A description is provided in **[Wiki section](https://github.com/AgatheArchet/smart-water-quality-monitoring/wiki) > 
