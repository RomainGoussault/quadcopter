/*
  IMU.cpp - Library to use the IMU 
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

#include "IMU.h"


IMU::IMU()
{  
}

void IMU::init()
{
	// Initialize device
	Serial.println("Initializing I2C devices...");
	accelgyro.initialize();

	// Check connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	delay(100); // Wait for sensor to stabilize

	accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);  //Set Starting angles
	accelgyro.setDLPFMode(3);  //Set Low Pass filter 

	accXangle = (atan2f(accX,accZ)+PI)*RAD_TO_DEG;
	accYangle = (atan2f(accY,accZ)+PI)*RAD_TO_DEG; //400

	//kalmanX.setAngle(accXangle); 
	//kalmanY.setAngle(accYangle);
	//kalmanZ.setAngle(accZangle);

	gyroXangle = accXangle;
	gyroYangle = accYangle;
	gyroZangle = 0;

	alpha_gyro = 0.995; 
	c = (1-alpha_gyro)*1;     
	compAngleX = accXangle;   
	compAngleY = accYangle;
	compAngleX0 = accXangle;   

	//Gyro Calibration: Rough guess of the gyro drift at startup
	float n = 200;

	float sX = 0.0;
	float sY = 0.0;
	float sZ = 0.0;	

	for (int i = 0; i < n; i++)
	{
		accelgyro.getRotation(&gyroX, &gyroY, &gyroZ);
		sX += accelgyro.getRotationX();
		sY += accelgyro.getRotationY();
		sZ += accelgyro.getRotationZ();
	}

	gyroXoffset = sX/n;
	gyroYoffset = sY/n;
	gyroZoffset = sZ/n;

	j=0;
}

bool IMU::processAngles(float angles[],float rates[])
{			
	accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);  //400µs
		
	//Filter
	accXf = filterX.update(accX);
	accYf = filterY.update(accY);
	accZf = filterZ.update(accZ);
	
	// Angular rates provided by gyro
	gyroXrate = (float) (gyroX-gyroXoffset)/131.0; //140 µsec on Arduino MEGA
	gyroYrate = -((float) (gyroY-gyroYoffset)/131.0);
	gyroZrate = ((float) (gyroZ-gyroZoffset)/131.0);

	//Angle provided by accelerometer
	//Time taken: 400 µsec on Arduino MEGA
	accYangle = (atan2f(accXf,accZf)+PI)*RAD_TO_DEG; // 
	accXangle = (atan2f(accYf,accZf)+PI)*RAD_TO_DEG; 
	
	//Integration of gyro rates to get the angles
	gyroXangle += gyroXrate*(float)(micros()-timer)/1000000;
	gyroYangle += gyroYrate*(float)(micros()-timer)/1000000;
	gyroZangle += gyroZrate*(float)(micros()-timer)/1000000;
	
	//Angle calculation through Complementary filter
	//Time taken: 200 µsec on Arduino MEGA
	dt = (float)(micros()-timer)/1000000.0;
	compAngleX = alpha_gyro*(compAngleX+(gyroXrate*dt))   +   c*accXangle; 
	compAngleY = alpha_gyro*(compAngleY+(gyroYrate*dt))  +   c*accYangle;
	//compAngleX0 = 0.997*(compAngleX0+(gyroXrate*(float)(micros()-timer)/1000000))   +   (1-0.997)*accXangle; 

	timer = micros(); 
	
	//45 deg rotation for roll and pitch (depending how your IMU is fixed to your quad)
	angles[0]=  -rac22* compAngleX + rac22*compAngleY + ROLL_OFFSET;
	angles[1]=  -rac22* compAngleX - rac22*compAngleY +2*rac22*PI*RAD_TO_DEG + PITCH_OFFSET;
	angles[2]=  gyroZangle;

	rates[0]=   -  rac22* gyroXrate + rac22*gyroYrate;
	rates[1]= - rac22* gyroXrate - rac22*gyroYrate;
	rates[2]=  gyroZrate;
		
	//////* Print Data  for vibration measurements*/ //Todo: Extract function
	//switch (j)
	  //{

	////	Frequency print
	  //case 1: 
		   //dtostrf(compAngleX - 180  ,6,2,StrAnglesvib);
		    //Serial.print(StrAnglesvib); 
		   //break;
	  //case 2:
		   //Serial.print("  ");
		   //break;
	  //case 3:
	  		//dtostrf(gyroXangle -180,6,2,StrAnglesvib);	
			//Serial.println(StrAnglesvib);
		   //j=0;
		   //break;
	  //}	   
	//j++;

	if ( abs(angles[0]) < ROLL_MAX_IMU && abs(angles[1]) < PITCH_MAX_IMU  )
	{
	return true;
	}
	else
	{
	return false;
	}
}
