////////////////////////////////////////////////////////////////////////////
//
//  This file is part of MPU9150Lib
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

//  DEVICE_TO_CAILIBRATE should be set to 0 for the IMU at 0x68 and 1 for the IMU at 0x69

#define  DEVICE_TO_CALIBRATE    1                       

MPU9150Lib MPU;                                            // the MPU9150Lib object

CALLIB_DATA calData;                                       // the calibration data

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (20)

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   5

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

void setup()
{
  calLibRead(DEVICE_TO_CALIBRATE, &calData);               // pick up existing accel data if there   

  calData.accelValid = false;
  calData.accelMinX = 0x7fff;                              // init accel cal data
  calData.accelMaxX = 0x8000;
  calData.accelMinY = 0x7fff;                              
  calData.accelMaxY = 0x8000;
  calData.accelMinZ = 0x7fff;                             
  calData.accelMaxZ = 0x8000;
    
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println("AccelCal9150 starting");
  Serial.println("Enter s to save current data to EEPROM");
  Wire.begin();

  MPU.selectDevice(DEVICE_TO_CALIBRATE);                   // select the correct device 
  MPU.useAccelCal(false);                                  // disable accel offsets
  MPU.init(MPU_UPDATE_RATE, 5, 1, MPU_LPF_RATE);           // start the MPU
  Serial.print("Calibrating device "); Serial.println(DEVICE_TO_CALIBRATE);
}

void loop()
{  
  boolean changed;
  
  MPU.selectDevice(DEVICE_TO_CALIBRATE);                   // not strictly needed here as the device never changes but good form
  if (MPU.read()) {                                        // get the latest data
    changed = false;
    if (MPU.m_rawAccel[VEC3_X] < calData.accelMinX) {
      calData.accelMinX = MPU.m_rawAccel[VEC3_X];
      changed = true;
    }
     if (MPU.m_rawAccel[VEC3_X] > calData.accelMaxX) {
      calData.accelMaxX = MPU.m_rawAccel[VEC3_X];
      changed = true;
    }
    if (MPU.m_rawAccel[VEC3_Y] < calData.accelMinY) {
      calData.accelMinY = MPU.m_rawAccel[VEC3_Y];
      changed = true;
    }
     if (MPU.m_rawAccel[VEC3_Y] > calData.accelMaxY) {
      calData.accelMaxY = MPU.m_rawAccel[VEC3_Y];
      changed = true;
    }
    if (MPU.m_rawAccel[VEC3_Z] < calData.accelMinZ) {
      calData.accelMinZ = MPU.m_rawAccel[VEC3_Z];
      changed = true;
    }
     if (MPU.m_rawAccel[VEC3_Z] > calData.accelMaxZ) {
      calData.accelMaxZ = MPU.m_rawAccel[VEC3_Z];
      changed = true;
    }
 
    if (changed) {
      Serial.println("-------");
      Serial.print("minX: "); Serial.print(calData.accelMinX);
      Serial.print(" maxX: "); Serial.print(calData.accelMaxX); Serial.println();
      Serial.print("minY: "); Serial.print(calData.accelMinY);
      Serial.print(" maxY: "); Serial.print(calData.accelMaxY); Serial.println();
      Serial.print("minZ: "); Serial.print(calData.accelMinZ);
      Serial.print(" maxZ: "); Serial.print(calData.accelMaxZ); Serial.println();
    }
  }
  
  if (Serial.available()) {
    if (Serial.read() == 's') {                  // save the data
      calData.accelValid = true;
      calLibWrite(DEVICE_TO_CALIBRATE, &calData);
      Serial.print("Accel cal data saved for device "); Serial.println(DEVICE_TO_CALIBRATE);
    }
  }
}
