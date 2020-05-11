/*********************************************************************
* GravityPh.h
*
* Copyright (C)    2017   [DFRobot](http://www.dfrobot.com),
* GitHub Link :https://github.com/DFRobot/watermonitor
* This Library is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Description:Monitoring water quality parameters ph
*
* Product Links：http://www.dfrobot.com.cn/goods-812.html
*
* Sensor driver pin：A2 
*
* author  :  Jason(jason.ling@dfrobot.com)
* version :  V1.0
* date    :  2017-04-07
**********************************************************************/

#pragma once
#include <Arduino.h>
#include "ISensor.h"

#define ReceivedBufferLength 10  // length of the Serial CMD buffer

class GravityPh:public ISensor
{ 
  public:
  	// ph sensor pin
  	int phSensorPin;
  
  	// sample and print interval
  	unsigned long samplingInterval;
   
 private:
    // main properties
    double _pHValue, _temperature;
  
    // data treatment
  	static const int _arrayLength = 5;
  	int _pHArray [_arrayLength];    //stores the average value of the sensor return data
    double _voltage, _averageVoltage, _sumVoltage; //stored for average measurement
  
    // data calibration
    double _acidVoltage, _neutralVoltage; //stored for pH probe calibration
    char _cmdReceivedBuffer[ReceivedBufferLength];  //stores the Serial CMD for calibration
    byte  _cmdReceivedBufferIndex;
  
  
 public:
  	GravityPh();
  	~GravityPh() {};
    
  	// initialization
  	void  setup ();
  
  	// update the sensor data
  	void  update ();
  
  	// get the sensor data
  	double getValue();
  
 private:
    // update subfunctions
    void calculateAnalogAverage();
    void calculatePh();
    void  calibration();

    // calibration by Serial command subfunctions 
    boolean cmdSerialDataAvailable();
    void    calibration(byte mode); // calibration process, wirte key parameters to EEPROM
    byte    cmdParse();
};
