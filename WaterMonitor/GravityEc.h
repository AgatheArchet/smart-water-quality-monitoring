/*********************************************************************
* GravityEc.h
*
* Copyright (C)    2017   [DFRobot](http://www.dfrobot.com),
* GitHub Link :https://github.com/DFRobot/watermonitor
* This Library is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Description:Monitoring water quality parameters Conductivity
*
* Product Links：http://www.dfrobot.com.cn/goods-882.html
* 
* Sensor driver pin：A1 (Can be modified in the .cpp file:ECsensorPin(A1);)
*
* author  :  Jason(jason.ling@dfrobot.com)
* version :  V1.0
* date    :  2017-04-17
**********************************************************************/

#pragma once
#include "GravityTemperature.h"
#include "ISensor.h"

// external GravityTemperature ecTemperature;
#define ReceivedBufferLength 10  //length of the Serial CMD buffer

class GravityEc:public ISensor
{
public:
	// Conductivity sensor pin
	int ecSensorPin;

	// Conductivity values
	double ECcurrent;

 // sample and print interval
  unsigned long AnalogSampleTime;
  unsigned long printTime;
  unsigned long AnalogSampleInterval;
  unsigned long printInterval ;


public:
	GravityEc(ISensor*);
	~GravityEc();

	// initialization
	void  setup ();

	// update the sensor data
	void  update ();

	// Get the sensor data
	double getValue();

private:
	// point to the temperature sensor pointer
	ISensor* _ecTemperature = NULL;


	static const int _numReadings = 5;
	unsigned int _readings[_numReadings] = { 0 };      // the readings from the analog input
	double _sumVoltage;
	unsigned long _AnalogValueTotal;      // the running total
	unsigned int _AnalogAverage;
	unsigned int _averageVoltage;



  float  _kvalue;
  float  _kvalueLow;
  float  _kvalueHigh;
  float  _voltage;
  float  _temperature;
  float  _rawEC;

  char   _cmdReceivedBuffer[ReceivedBufferLength];  //store the Serial CMD
  byte   _cmdReceivedBufferIndex;

private:
	// Calculate the average
  void calculateAnalogAverage();

	// Calculate the conductivity
	void calculateEc();

 void calibration(byte mode);
};
