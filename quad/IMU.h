/*
  Motors.h - Library for controlling a set of Quadcopter motors (aka motors)
  Created by Myles Grant <myles@mylesgrant.com>
  See also: https://github.com/grantmd/QuadCopter
  
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include <Utils.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"


#define  ROLL_MAX_IMU  30
#define  PITCH_MAX_IMU 30

#define  ROLL_OFFSET -0.64
#define  PITCH_OFFSET 0.07
#define  YAW_OFFSET 9.70

//#define  rac22 0.707

class IMU
{

  public:

  IMU();
  
  //initialize the IMU
  void init();  
 
  //Process the angles
  bool processAngles(float angles[] );
    



    
  private:
    
	Kalman kalmanX; // Create the Kalman instances
	Kalman kalmanY;

	/* IMU Data */
	int16_t accX, accY, accZ;
	int16_t gyroX, gyroY, gyroZ;

	double accXangle, accYangle; // Angle calculate using the accelerometer
	double gyroXangle, gyroYangle; // Angle calculate using the gyro
	double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter
	MPU6050 accelgyro;
	uint32_t timer;

};

#endif
