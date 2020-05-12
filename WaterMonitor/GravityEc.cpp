/*********************************************************************
* GravityEc.cpp
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
* Sensor driver pin：A1 (ecSensorPin(A1))
*
* author  :  Jason(jason.ling@dfrobot.com)
* version :  V1.0
* date    :  2017-04-17
**********************************************************************/

#include "GravityEc.h"
#include "Arduino.h"
#include <EEPROM.h>

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define KVALUEADDR 0x0A    //the start address of the K value stored in the EEPROM
#define RES2 820.0
#define ECREF 200.0


GravityEc::GravityEc(ISensor* temp) :ecSensorPin(A1), ECcurrent(0), _AnalogAverage(0),
_AnalogValueTotal(0), _averageVoltage(0), AnalogSampleTime(0), printTime(0),_sumVoltage(0),
AnalogSampleInterval(25),printInterval(700), 
_kvalue(1.0),_kvalueLow(1.0), _kvalueHigh(1.0), _voltage(0.0), _temperature(25.0)
{
	this->_ecTemperature = temp;
}


GravityEc::~GravityEc()
{
}


//********************************************************************************************
// function name: setup ()
// Function Description: Initializes the sensor
//********************************************************************************************
void GravityEc::setup()
{
	pinMode(ecSensorPin, INPUT);
  
	for (byte thisReading = 0; thisReading < _numReadings; thisReading++)
		_readings[thisReading] = 0;

  EEPROM_read(KVALUEADDR, this->_kvalueLow);        //read the calibrated K value from EEPROM
  if(EEPROM.read(KVALUEADDR)==0xFF && EEPROM.read(KVALUEADDR+1)==0xFF && EEPROM.read(KVALUEADDR+2)==0xFF && EEPROM.read(KVALUEADDR+3)==0xFF)
  {
    this->_kvalueLow = 1.0;                       // For new EEPROM, write default value( K = 1.0) to EEPROM
    EEPROM_write(KVALUEADDR, this->_kvalueLow);
  }
  EEPROM_read(KVALUEADDR+4, this->_kvalueHigh);     //read the calibrated K value from EEPRM
  if(EEPROM.read(KVALUEADDR+4)==0xFF && EEPROM.read(KVALUEADDR+5)==0xFF && EEPROM.read(KVALUEADDR+6)==0xFF && EEPROM.read(KVALUEADDR+7)==0xFF){
    this->_kvalueHigh = 1.0;                      // For new EEPROM, write default value( K = 1.0) to EEPROM
    EEPROM_write(KVALUEADDR+4, this->_kvalueHigh);
  }
  this->_kvalue =  this->_kvalueLow;                // set default K value: K = kvalueLow 
  
  Serial.print("* EC Saved values with previous calibration : ");
  Serial.print(" kvalueLow   : ");
  Serial.print(this->_kvalueLow);
  Serial.print("      kvalueHigh     : ");
  Serial.println(this->_kvalueHigh);
}


//********************************************************************************************
// function name: update ()
// Function Description: Update the sensor value
//********************************************************************************************
void GravityEc::update()
{
	calculateAnalogAverage();
	calculateEc();
}


//********************************************************************************************
// function name: getValue ()
// Function Description: Returns the sensor data
//********************************************************************************************
double GravityEc::getValue()
{
 return this->ECcurrent;
}


//********************************************************************************************
// function name: calculateAnalogAverage ()
// Function Description: Calculates the average voltage
//********************************************************************************************
void GravityEc::calculateAnalogAverage()
{
  static int ecArrayIndex = 0;
	if (millis() - AnalogSampleTime >= AnalogSampleInterval)
	{
		AnalogSampleTime = millis();
		_readings[ecArrayIndex++] = analogRead(ecSensorPin)/1024.0*5000;
		if (ecArrayIndex == _numReadings)
		{
			ecArrayIndex = 0;
			for (int i = 0; i < _numReadings; i++)
				this->_sumVoltage += _readings[i];
			this->_AnalogAverage = this->_sumVoltage / _numReadings;
			this->_sumVoltage = 0;
		}
	}
}


//********************************************************************************************
// function name: calculateAnalogAverage ()
// Function Description: Calculate the conductivity
//********************************************************************************************
void GravityEc::calculateEc()
{
  float value = 0,valueTemp = 0;
  this->_voltage = this->_AnalogAverage;
  this->_rawEC = 1000*this->_voltage/RES2/ECREF;
  valueTemp = this->_rawEC * this->_kvalue;
  //automatic shift process
  //First Range:(0,2); Second Range:(2,20)
  if(valueTemp > 2.5){
      this->_kvalue = this->_kvalueHigh;
  }else if(valueTemp < 2.0){
      this->_kvalue = this->_kvalueLow;
  }

  value = this->_rawEC * this->_kvalue;             //calculate the EC value after automatic shift
  value = value / (1.0+0.0185*(this->_temperature-25.0));  //temperature compensation
  this->ECcurrent = value;                           //store the EC value for Serial CMD calibration
}


void GravityEc::calibration(byte mode)
{
  char *receivedBufferPtr;
  static boolean ecCalibrationFinish  = 0;
  static boolean enterCalibrationFlag = 0;
  static float compECsolution;
  float KValueTemp;
  switch(mode){
      case 0:
      if(enterCalibrationFlag){
          Serial.println(F(">>>Command Error<<<"));
      }
      break;
      case 4:
      enterCalibrationFlag = 1;
      ecCalibrationFinish  = 0;
      Serial.println();
      Serial.println(F(">>>Enter EC Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the 1413us/cm or 12.88ms/cm buffer solution<<<"));
      Serial.println();
      break;
      case 5:
      if(enterCalibrationFlag){
          if((this->_rawEC>0.9)&&(this->_rawEC<1.9)){                         //recognize 1.413us/cm buffer solution
              compECsolution = 1.413*(1.0+0.0185*(this->_temperature-25.0));  //temperature compensation
          }else if((this->_rawEC>9)&&(this->_rawEC<16.8)){                    //recognize 12.88ms/cm buffer solution
              compECsolution = 12.88*(1.0+0.0185*(this->_temperature-25.0));  //temperature compensation
          }else{
              Serial.print(F(">>>Buffer Solution Error Try Again<<<   "));
              ecCalibrationFinish = 0;
          }
          KValueTemp = RES2*ECREF*compECsolution/1000.0/this->_voltage;       //calibrate the k value
          if((KValueTemp>0.5) && (KValueTemp<1.5)){
              Serial.println();
              Serial.print(F(">>>Successful,K:"));
              Serial.print(KValueTemp);
              Serial.println(F(", Send EXITEC to Save and Exit<<<"));
              if((this->_rawEC>0.9)&&(this->_rawEC<1.9)){
                  this->_kvalueLow =  KValueTemp;
              }else if((this->_rawEC>9)&&(this->_rawEC<16.8)){
                  this->_kvalueHigh =  KValueTemp;
              }
              ecCalibrationFinish = 1;
        }
          else{
              Serial.println();
              Serial.println(F(">>>Failed,Try Again<<<"));
              Serial.println();
              ecCalibrationFinish = 0;
          }
      }
      break;
      case 6:
      if(enterCalibrationFlag){
              Serial.println();
              if(ecCalibrationFinish){   
                  if((this->_rawEC>0.9)&&(this->_rawEC<1.9)){
                      EEPROM_write(KVALUEADDR, this->_kvalueLow);
                  }else if((this->_rawEC>9)&&(this->_rawEC<16.8)){
                      EEPROM_write(KVALUEADDR+4, this->_kvalueHigh);
                  }
                  Serial.print(F(">>>Calibration Successful"));
              }else{
                  Serial.print(F(">>>Calibration Failed"));
              }
              Serial.println(F(",Exit EC Calibration Mode<<<"));
              Serial.println();
              ecCalibrationFinish  = 0;
              enterCalibrationFlag = 0;
      }
      break;
  }
} 
