/*********************************************************************************************
* GravitySensorHub.cpp
*
* Copyright (C)    2017   [DFRobot](http://www.dfrobot.com),
* GitHub Link :https://github.com/DFRobot/watermonitor
* This Library is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Description:
*
* author  :  Jason(jason.ling@dfrobot.com)
* version :  V1.0
* date    :  2017-04-12
*********************************************************************************************/


#include "GravitySensorHub.h"
#include "GravityPh.h"
// #include "GravityOrp.h"
#include "GravityEc.h"
//#include "GravityTemperature.h"
// #include "SensorDo.h"

//********************************************************************************************
// function name: sensors []
// Function Description: Store the array of sensors
// Parameters: 0 ph sensor
// Parameters: 1 temperature sensor
// Parameters: 2 Dissolved oxygen sensor
// Parameters: 3 Conductivity sensor
// Parameters: 4 Redox potential sensor
//********************************************************************************************

GravitySensorHub::GravitySensorHub()
{
	for (size_t i = 0; i < this->SensorCount; i++)
	{
		this->sensors[i] = NULL;
	}

this->sensors[0] = new GravityPh();
//this->sensors[1] = new GravityTemperature(5);
//	this->sensors[2] = new SensorDo();
this->sensors[3] = new GravityEc(this->sensors[1]);
//	this->sensors[4] = new GravityOrp();

}

//********************************************************************************************
// function name: ~ GravitySensorHub ()
// Function Description: Destructor, frees all sensors
//********************************************************************************************
GravitySensorHub::~GravitySensorHub()
{
	for (size_t i = 0; i < SensorCount; i++)
	{
		if (this->sensors[i])
		{
			delete this->sensors[i];
		}
	}
}


//********************************************************************************************
// function name: setup ()
// Function Description: Initializes all sensors
//********************************************************************************************
void GravitySensorHub::setup()
{
	for (size_t i = 0; i < SensorCount; i++)
	{
		if (this->sensors[i])
		{
			this->sensors[i]->setup();
		}
	}
}


//********************************************************************************************
// function name: update ()
// Function Description: Updates all sensor values
//********************************************************************************************
void GravitySensorHub::update()
{
	for (size_t i = 0; i < SensorCount; i++)
	{
		if (this->sensors[i])
		{
			this->sensors[i]->update();
		}
	}
}

//********************************************************************************************
// function name: getValueBySensorNumber ()
// Function Description: Get the sensor data
// Parameters: 0 ph sensor
// Parameters: 1 temperature sensor
// Parameters: 2 Dissolved oxygen sensor
// Parameters: 3 Conductivity sensor
// Parameters: 4 Redox potential sensor
// Return Value: Returns the acquired sensor data
//********************************************************************************************
double GravitySensorHub::getValueBySensorNumber(int num)
{
	if (num >= SensorCount)
	{
		return 0;
	}
	return this->sensors[num]->getValue();
}

void GravitySensorHub::calibrate()
{   
    if(cmdSerialDataAvailable() > 0)
    {
        byte option = cmdParse(); // if received Serial CMD from the serial monitor, enter into the calibration mode
        Serial.print("  option :"); Serial.println(option);
        if (option==0)
           Serial.println(F(">>>Command Error<<<"));
        else if (option >=1 && option<=3)
            if  (this->sensors[0] == NULL)
               Serial.println(F(">>>pH Sensor does not exit<<<"));
            else
                this->sensors[0]->calibration(option);
        else if(option >=4 && option<=6)
            if  (this->sensors[3] == NULL)
               Serial.println(F(">>>EC Sensor does not exit<<<"));
            else
               this->sensors[3]->calibration(option);
    }
}

boolean GravitySensorHub::cmdSerialDataAvailable()
{
    char cmdReceivedChar;
    static unsigned long cmdReceivedTimeOut = millis();
    while (Serial.available()>0) 
    {
        if(millis() - cmdReceivedTimeOut > 500U){
            this->_cmdReceivedBufferIndex = 0;
            memset(this->_cmdReceivedBuffer,0,(ReceivedBufferLength));
        }
        cmdReceivedTimeOut = millis();
        cmdReceivedChar = Serial.read();
        if(cmdReceivedChar == '\n' || this->_cmdReceivedBufferIndex==ReceivedBufferLength-1)
        {
            this->_cmdReceivedBufferIndex = 0;
            strupr(this->_cmdReceivedBuffer);
            return true;
        }else{
            this->_cmdReceivedBuffer[this->_cmdReceivedBufferIndex] = cmdReceivedChar;
            this->_cmdReceivedBufferIndex++;
        }
    }
    return false;
}

byte GravitySensorHub::cmdParse()
{
    byte modeIndex = 0;
    // ph calibration
    if(strstr(this->_cmdReceivedBuffer, "ENTERPH")      != NULL)
        modeIndex = 1;
    else if(strstr(this->_cmdReceivedBuffer, "EXITPH") != NULL)
        modeIndex = 3;
    else if(strstr(this->_cmdReceivedBuffer, "CALPH")  != NULL)
        modeIndex = 2;
    // ec calibration
    else if(strstr(this->_cmdReceivedBuffer, "ENTEREC")     != NULL)
        modeIndex = 4;
    else if(strstr(this->_cmdReceivedBuffer, "EXITEC") != NULL)
        modeIndex = 6;
    else if(strstr(this->_cmdReceivedBuffer, "CALEC")  != NULL)
        modeIndex = 5;
    return modeIndex;
}
