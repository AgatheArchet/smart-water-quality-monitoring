/*********************************************************************
 * WaterMonitor.ino
 *
 * Copyright (C)    2017   [DFRobot](http://www.dfrobot.com)
 * GitHub Link :https://github.com/DFRobot/watermonitor
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Description:
 * This sample code is mainly used to monitor water quality
 * including ph, temperature, dissolved oxygen, ec and orp,etc.
 *
 * Software Environment: Arduino IDE 1.8.2
 * Software download link: https://www.arduino.cc/en/Main/Software
 *
 * Install the library file：
 * Copy the files from the github repository folder libraries to the libraries
 * in the Arduino IDE 1.8.2 installation directory
 *
 * Hardware platform   : Arduino M0 Or Arduino Mega2560
 * Sensor pin:
 * EC  : A1
 * PH  : A2
 * ORP : A3
 * RTC : I2C
 * DO  : Serial port Rx(0),Tx(1)
 * GravityDO：A4
 * temperature:D5
 *
 * SD card attached to SPI bus as follows:
 * Mega:  MOSI - pin 51, MISO - pin 50, CLK - pin 52, CS - pin 53
 * and pin #53 (SS) must be an output
 * M0:   Onboard SPI pin,CS - pin 4 (CS pin can be changed)
 *
 * author  :  Jason(jason.ling@dfrobot.com)
 * version :  V1.0
 * date    :  2017-04-06
 **********************************************************************/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "GravitySensorHub.h"
#include "GravityRtc.h"
#include "OneWire.h"
#include "SdService.h"
#include "Debug.h"
#include <SoftwareSerial.h>


// clock module
GravityRtc rtc;

// sensor monitor
GravitySensorHub sensorHub;
SdService sdService = SdService(sensorHub.sensors);

void setup() {
	Serial.begin(115200);

 // clock time set-up
	rtc.setup();
  rtc.adjustRtc(F(__DATE__), F(__TIME__)); //Set time given by computer
  //rtc.adjustRtc(2020,1,31,4,12,46,0);  //Set time: (year,month,day,dayOfWeek,hour,minute,second), here : 1/31/2020, Wenesday, 12:46:00
	
	// Sensors and record file initialization
	sensorHub.setup();
	sdService.setup();
  sdService.clearPreviousData();
 
  Serial.println(" ");
  }

//********************************************************************************************
// function name: sensorHub.getValueBySensorNumber (0)
// Function Description: Get the sensor's values, and the different parameters represent the acquisition of different sensor data     
// Parameters: 0 ph value  
// Parameters: 1 temperature value    
// Parameters: 2 Dissolved Oxygen
// Parameters: 3 Conductivity
// Parameters: 4 Redox potential
// return value: returns a double type of data

// Function Name: calibrate()
// Function Declaration: Launches verification steps to enable any calibration (from a Serial command)
// ENTERPH : enters pH calibration mode
// CALPH : detects the buffer solution 4.0 or 7.0
// EXITPH : exits calibration mode and saves the values ( /!\ will not be saved otherwise)
// ENTEREC : enters EC calibration mode
// CALEC : detects the buffer solution 1413 µS/cm or 12.88 mS/cm
// EXITEC : exits calibration mode and saves the values ( /!\ will not be saved otherwise)
//********************************************************************************************

unsigned long updateTime = 0;
int i =0;

void loop() {
  // updates all modules
  rtc.update();
	sensorHub.update();
	sdService.update();
  sensorHub.calibrate(); // if calibration required at any time
  
  if(millis() - updateTime > 900) // 2 seconds between each Serial.print()
  {
     // change the code in this part if needed
     updateTime = millis(); 
     Serial.print("M");Serial.print(rtc.month);
     Serial.print("d");Serial.print(rtc.day);
     Serial.print("y"); Serial.print(rtc.year);
     Serial.print("h"); Serial.print(rtc.hour);
     Serial.print("m");Serial.print(rtc.minute);
     Serial.print("s"); Serial.print(rtc.second);
     delay(300);
     Serial.print("pH"); Serial.print(sensorHub.getValueBySensorNumber(0));
     Serial.print("tp"); Serial.print("25.0");  //Serial.print(sensorHub.getValueBySensorNumber(1));
     Serial.print("EC"); Serial.print(sensorHub.getValueBySensorNumber(3));
     delay(300);
     Serial.print("Do");Serial.print("0.00"); //Serial.print(sensorHub.getValueBySensorNumber(2));
     Serial.print("Or"); Serial.print("0.00");//Serial.print(sensorHub.getValueBySensorNumber(4));
     delay(300); 
  }
  
}



//* ***************************** Print the relevant debugging information ************** ************ * /
// Note: Arduino M0 need to replace Serial with SerialUSB when printing debugging information

// ************************* Serial debugging ******************
/*
    i++;
    updateTime = millis();
    Serial.print(i); 
    
    Serial.print("   Date :  "); Serial.print(rtc.month);  
    Serial.print("/"); Serial.print(rtc.day); 
    Serial.print("/"); Serial.print(rtc.year); 
    Serial.print(","); Serial.print(rtc.week); 
    Serial.print(","); Serial.print(rtc.hour); 
    Serial.print(":"); Serial.print(rtc.minute); 
    Serial.print(":"); Serial.print(rtc.second); 
    
    Serial.print(F(" pH: "));
    Serial.print(sensorHub.getValueBySensorNumber(0));
    Serial.print(F("  Temp: "));
    Serial.print(sensorHub.getValueBySensorNumber(1));
    Serial.print(F("  Do: "));
    Serial.print(sensorHub.getValueBySensorNumber(2));
    Serial.print(" mg/L");
    Serial.print(F("  Ec: "));
    Serial.print(sensorHub.getValueBySensorNumber(3));
    Serial.print(" mS/cm");
    Serial.print(F("  Orp: "));
    Serial.print(sensorHub.getValueBySensorNumber(4)); 
    Serial.println(" mV");
*/

// ********************************** time *********************************
/*
    Serial.print("   Year = ");//year
    Serial.print(rtc.year);
    Serial.print("   Month = ");//month
    Serial.print(rtc.month);
    Serial.print("   Day = ");//day
    Serial.print(rtc.day);
    Serial.print("   Week = ");//week
    Serial.print(rtc.week);
    Serial.print("   Hour = ");//hour
    Serial.print(rtc.hour);
    Serial.print("   Minute = ");//minute
    Serial.print(rtc.minute);
    Serial.print("   Second = ");//second
    Serial.println(rtc.second);
*/

// *************************** calibration debugging *****************************
/*
    i++;
    updateTime = millis();
    Serial.print(i); 
    
    Serial.print("   Date :  "); Serial.print(rtc.month);  
    Serial.print("/"); Serial.print(rtc.day); 
    Serial.print("/"); Serial.print(rtc.year); 
    Serial.print(","); Serial.print(rtc.week); 
    Serial.print(","); Serial.print(rtc.hour); 
    Serial.print(":"); Serial.print(rtc.minute); 
    Serial.print(":"); Serial.print(rtc.second); 
    
    Serial.print("   pH : "); 
    Serial.print(sensorHub.getValueBySensorNumber(0));
    Serial.print("  raw voltage : "); 
    double value = analogRead(A2);
    Serial.print(value);
    Serial.print("  volatge  :  ");
    Serial.println(value/1024.0*5000);
    
    Serial.print("   EC : "); 
    Serial.print(sensorHub.getValueBySensorNumber(3)); 
    Serial.print(" mS/cm");
    Serial.print("  raw voltage : "); 
    double value = analogRead(A1);
    Serial.print(value);
    Serial.print("  volatge  :  ");
    Serial.println(value/1024.0*5000);
*/

// *************************** bluetooth sending debugging *****************************
/* 

     updateTime = millis();
     Serial.print("M");Serial.print(rtc.month);
     Serial.print("d");Serial.print(rtc.day);
     Serial.print("y"); Serial.print(rtc.year);
     Serial.print("h"); Serial.print(rtc.hour);
     Serial.print("m");Serial.print(rtc.minute);
     Serial.print("s"); Serial.print(rtc.second);
     delay(500);
     Serial.print("pH"); Serial.print(sensorHub.getValueBySensorNumber(0));
     Serial.print("tp"); Serial.print("25.0");  //Serial.print(sensorHub.getValueBySensorNumber(1));
     Serial.print("EC"); Serial.print(sensorHub.getValueBySensorNumber(3));
     delay(500);
     Serial.print("Do");Serial.print("0.00"); //Serial.print(sensorHub.getValueBySensorNumber(2));
     Serial.print("Or"); Serial.print("0.00");//Serial.print(sensorHub.getValueBySensorNumber(4));
     delay(500);
 */
