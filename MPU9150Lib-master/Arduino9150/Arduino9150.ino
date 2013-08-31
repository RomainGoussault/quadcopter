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

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69

#define  DEVICE_TO_USE    1

MPU9150Lib MPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   5 //40

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.print("Arduino9150 starting using device "); Serial.println(DEVICE_TO_USE);
  Wire.begin();
  MPU.selectDevice(DEVICE_TO_USE);                        // only really necessary if using device 1
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
}

void loop()
{  
  MPU.selectDevice(DEVICE_TO_USE);                         // only needed if device has changed since init but good form anyway
  if (MPU.read()) {                                        // get the latest data if ready yet
//  MPU.printQuaternion(MPU.m_rawQuaternion);              // print the raw quaternion from the dmp
//  MPU.printVector(MPU.m_rawMag);                         // print the raw mag data
//  MPU.printVector(MPU.m_rawAccel);                       // print the raw accel data
//  MPU.printAngles(MPU.m_dmpEulerPose);                   // the Euler angles from the dmp quaternion
//  MPU.printVector(MPU.m_calAccel);                       // print the calibrated accel data
//  MPU.printVector(MPU.m_calMag);                         // print the calibrated mag data
    MPU.printAngles(MPU.m_fusedEulerPose);                 // print the output of the data fusion
    Serial.println();
  }
}
