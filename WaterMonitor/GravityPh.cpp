/*********************************************************************
* GravityPh.cpp
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
* Sensor driver pin：A2 (phSensorPin(A2))
*
* author  :  Jason(jason.ling@dfrobot.com)
* version :  V1.0
* date    :  2017-04-07
**********************************************************************/

#include "GravityPh.h"
#include <EEPROM.h>

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define PHVALUEADDR 0x00    //the start address of the pH calibration parameters stored in the EEPROM


GravityPh::GravityPh():phSensorPin(A2), samplingInterval(25),_pHValue(0),_averageVoltage(0),
_voltage(1500), _sumVoltage(0),_acidVoltage(2032.44), _neutralVoltage(1500.0), _temperature(25.0)
{
}


void GravityPh::setup()
{
    pinMode(phSensorPin, INPUT);
    
       EEPROM_read(PHVALUEADDR, this->_neutralVoltage);  //load the neutral (pH = 7.0)voltage of the pH board from the EEPROM
      if(EEPROM.read(PHVALUEADDR)==0xFF && EEPROM.read(PHVALUEADDR+1)==0xFF && EEPROM.read(PHVALUEADDR+2)==0xFF && EEPROM.read(PHVALUEADDR+3)==0xFF)
      {
          this->_neutralVoltage = 1500.0;  // new EEPROM, write typical voltage
          EEPROM_write(PHVALUEADDR, this->_neutralVoltage);
      }
      EEPROM_read(PHVALUEADDR+4, this->_acidVoltage);//load the acid (pH = 4.0) voltage of the pH board from the EEPROM
      if(EEPROM.read(PHVALUEADDR+4)==0xFF && EEPROM.read(PHVALUEADDR+5)==0xFF && EEPROM.read(PHVALUEADDR+6)==0xFF && EEPROM.read(PHVALUEADDR+7)==0xFF)
      {
          this->_acidVoltage = 2032.44;  // new EEPROM, write typical voltage
          EEPROM_write(PHVALUEADDR+4, this->_acidVoltage);
      }  
}


void GravityPh::update()
{
    calculateAnalogAverage();
    calculatePh();
    //calibration();
}


void GravityPh::calculateAnalogAverage()
{
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static int pHArrayIndex = 0;
    if (millis() - samplingTime > samplingInterval)
    {
       samplingTime = millis();
        _pHArray[pHArrayIndex++] = analogRead(this->phSensorPin)/1024.0*5000;
      
        if (pHArrayIndex == _arrayLength)   // 5 * 20 = 100ms
        {
             pHArrayIndex = 0;
             for (int i = 0; i < _arrayLength; i++)
             this->_sumVoltage += _pHArray[i];
             this->_averageVoltage = this->_sumVoltage / _arrayLength;
             this->_sumVoltage = 0;
         }
    }
}


void GravityPh::calculatePh()
{
    this->_voltage = this->_averageVoltage;
    float slope = (7.0-4.0)/((this->_neutralVoltage-1500.0)/3.0 - (this->_acidVoltage-1500.0)/3.0);  // two point: (_neutralVoltage,7.0),(__acidVoltage,4.0)
    float intercept =  7.0 - slope*(this->_neutralVoltage-1500.0)/3.0;
    this->_pHValue = slope*(this->_voltage-1500.0)/3.0+intercept;  //y = k*x + b
}


double GravityPh::getValue()
{
	 return this->_pHValue;
}

void GravityPh::calibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean phCalibrationFinish  = 0;
    static boolean enterCalibrationFlag = 0;
    switch(mode){
        case 0:
        if(enterCalibrationFlag)
        {
            Serial.println(F(">>>Command Error<<<"));
        }
        break;

        case 1:
        enterCalibrationFlag = 1;
        phCalibrationFinish  = 0;
        Serial.println();
        Serial.println(F(">>>Enter PH Calibration Mode<<<"));
        Serial.println(F(">>>Please put the probe into the 4.0 or 7.0 standard buffer solution<<<"));
        Serial.println();
        break;

        case 2:
        if(enterCalibrationFlag){
            if((this->_voltage>1322)&&(this->_voltage<1678))
            {        // buffer solution:7.0{
                Serial.println();
                Serial.print(F(">>>Buffer Solution:7.0"));
                this->_neutralVoltage =  this->_voltage;
                Serial.println(F(",Send EXITPH to Save and Exit<<<"));
                Serial.println();
                phCalibrationFinish = 1;
            }else if((this->_voltage>1854)&&(this->_voltage<2210)){  //buffer solution:4.0
                Serial.println();
                Serial.print(F(">>>Buffer Solution:4.0"));
                this->_acidVoltage =  this->_voltage;
                Serial.println(F(",Send EXITPH to Save and Exit<<<")); 
                Serial.println();
                phCalibrationFinish = 1;
            }else{
                Serial.println();
                Serial.print(F(">>>Buffer Solution Error Try Again<<<"));
                Serial.println();                                    // not buffer solution or faulty operation
                phCalibrationFinish = 0;
            }
        }
        break;

        case 3:
        if(enterCalibrationFlag)
        {
            Serial.println();
            if(phCalibrationFinish)
            {
                if((this->_voltage>1322)&&(this->_voltage<1678))
                {
                    EEPROM_write(PHVALUEADDR, this->_neutralVoltage);
                }else if((this->_voltage>1854)&&(this->_voltage<2210))
                          {
                             EEPROM_write(PHVALUEADDR+4, this->_acidVoltage);
                           }
                Serial.print(F(">>>Calibration Successful"));
            }else{
                Serial.print(F(">>>Calibration Failed"));
            }
            Serial.println(F(",Exit PH Calibration Mode<<<"));
            Serial.println();
            phCalibrationFinish  = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}
