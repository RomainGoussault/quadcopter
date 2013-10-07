/*
  Motors.cpp - Library for controlling a set of Quadcopter motors (aka motors)
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


#include "IMU.h"




IMU::IMU(){  
 
}

void IMU::init()
{
	
    
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    delay(100); // Wait for sensor to stabilize
  
  /* Set kalman and gyro starting angle */
 accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2	
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
}



bool IMU::processAngles(float angles[])
{
	
    // read raw accel/gyro measurements from device
	accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  		
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  	
  	
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  
   
  kalAngleX = (kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000)    -PI*RAD_TO_DEG); // Calculate the angle using a Kalman filter
  kalAngleY = (kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000)     -PI*RAD_TO_DEG);
  
    
   		

		
  timer = micros();
  
  	//45deg rotation for roll and pitch
	angles[0]=     -  rac22* kalAngleX + rac22*kalAngleY;
	angles[1]=         +rac22* kalAngleX + rac22*kalAngleY;
	angles[2]=0;//m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE  -  YAW_OFFSET;
  
  /* Print Data */


  //Serial.print(angles[0]);Serial.print("\t");
  //Serial.print(angles[1]);Serial.print("\t");
  
    //Serial.print(kalAngleX);Serial.print("\t");
     //Serial.print(kalAngleY);Serial.print("\t");
    
 
      	
   if ( abs(angles[0]) < ROLL_MAX_IMU && abs(angles[1]) < PITCH_MAX_IMU  )
   {
	   return true;
   }
   else
   {
	   return false;
   }
   
  
 
		
}








