/*
  IMU.h - Library to use the IMU 
  Created by Romain Goussault <romain.goussault@gmail.com>
  
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  afloat with this program. If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include <Utils.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"
#include "Filter.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define  ROLL_MAX_IMU  30
#define  PITCH_MAX_IMU 30

#define  ROLL_OFFSET -0.16
#define  PITCH_OFFSET 2.10
#define  YAW_OFFSET 0


class IMU
{
  public:

  IMU();
  void init();  
  bool processAngles(float angles[],float rates[] );

    
  private:
    
	//Kalman kalmanX; // Create the Kalman instances
	//Kalman kalmanY;
	//Kalman kalmanZ;
	/* IMU Data */
	int16_t accX, accY, accZ;
	int16_t gyroX, gyroY, gyroZ;

	float accXangle, accYangle, accZangle; // Angle calculated using the accelerometer
	float gyroXangle, gyroYangle, gyroZangle; // Angle calculated using the gyro
	float kalAngleX, kalAngleY, kalAngleZ; // Angle calculated using a Kalman filter
	float compAngleX, compAngleY, compAngleX0;

	MPU6050 accelgyro;
	uint32_t timer;
	float gyroXoffset, gyroYoffset, gyroZoffset;
	
	float gyroXrate ;
	float gyroYrate ;
	float gyroZrate;
	
    //float xv[NZEROS+1], yv[NPOLES+1];
     //float xv1[NZEROS+1], yv1[NPOLES+1];  
	float accXf;
	float accYf;
	float accZf;
	
	Filter filterX;
	Filter filterY;	
	Filter filterZ;
	
	float alpha_gyro;
    float c;
    float dt;
	char StrAnglesvib[7];
	int j;
};

#endif
