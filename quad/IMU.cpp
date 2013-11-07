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
  afloat with this program. If not, see <http://www.gnu.org/licenses/>. 
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
  

 accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);  //Set Starting angles
 accelgyro.setDLPFMode(2);  //Set Low Pass filter 


	accXangle = (atan2f(accX,accZ)+PI)*RAD_TO_DEG;
	accYangle = (atan2f(accY,accZ)+PI)*RAD_TO_DEG; //400
  
  //kalmanX.setAngle(accXangle); // Set starting angle
  //kalmanY.setAngle(accYangle);
  //kalmanZ.setAngle(accZangle);
    
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  gyroZangle = 0;

  alpha_gyro = 0.995; 
  compAngleX = accXangle;   
  compAngleY = accYangle;
 compAngleX0 = accXangle;   

  
  
	//Gyro Calibration
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
	
	c = (1-alpha_gyro)*1;
	
	j=0;
	
	//Filter init


}



bool IMU::processAngles(float angles[],float rates[])
{			
	accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);  //400
		  //double start = micros();
	//Filter
	accXf = filterX.update(accX);
	accYf = filterY.update(accY);
	accZf = filterZ.update(accZ);

//double delta = micros() - start;
//Serial.print("    df: ");	
//Serial.println(delta);
	
	
	// ANGULAR RATES
	gyroXrate = -(float) (gyroX-gyroXoffset)/131.0;  //140
	gyroYrate = ((float) (gyroY-gyroYoffset)/131.0);
	gyroZrate = ((float) (gyroZ-gyroZoffset)/131.0);

	//ACC ANGLES
	accXangle = (atan2f(accXf,accZf)+PI)*RAD_TO_DEG;
	accYangle = (atan2f(accYf,accZf)+PI)*RAD_TO_DEG; //400
	
	
	// GYRO ANGLES
	//gyroXangle += gyroXrate*(float)(micros()-timer)/1000000;
	//gyroYangle += gyroYrate*(float)(micros()-timer)/1000000;
	//gyroZangle += gyroZrate*(float)(micros()-timer)/1000000;
	
	
	//Complementary filter  //200
	dt = (float)(micros()-timer)/1000000.0;
	compAngleX = alpha_gyro*(compAngleX+(gyroXrate*dt))   +   c*accXangle; // Calculate the angle using a Complimentary filter 
	//compAngleX0 = 0.997*(compAngleX0+(gyroXrate*(float)(micros()-timer)/1000000))   +   (1-0.997)*accXangle; // Calculate the angle using a Complimentary filter 
	compAngleY = alpha_gyro*(compAngleY+(gyroYrate*dt))  +   c*accYangle; // Calculate the angle using a Complimentary filter 

	timer = micros(); 
	
	
	
	//45 deg rotation for roll and pitch
	angles[0]=  -rac22* compAngleX + rac22*compAngleY + ROLL_OFFSET;
	angles[1]=  -rac22* compAngleX - rac22*compAngleY +2*rac22*PI*RAD_TO_DEG + PITCH_OFFSET;
	angles[2]=  gyroZangle;

	rates[0]=   -  rac22* gyroXrate + rac22*gyroYrate;
	rates[1]=  -rac22* gyroXrate - rac22*gyroYrate;
	rates[2]=  gyroZrate;
	
	

	


	
	//////* Print Data  for vib measurements*/
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
	  		//dtostrf(accXangle -180,6,2,StrAnglesvib);	
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

//void IMU::Filter()
//{ 
		////xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; 
        ////xv[4] = accXangle / GAIN;
        ////yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; 
        ////yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
                     ////+ ( -0.4806953276 * yv[0]) + (  2.2730179578 * yv[1])
                     ////+ ( -4.0680437467 * yv[2]) + (  3.2713933380 * yv[3]);
        ////accFilter =yv[4];
//xv[0] = xv[1]; xv[1] = xv[2]; 
        //xv[2] = accX / GAIN;
        //yv[0] = yv[1]; yv[1] = yv[2]; 
        //yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
                     //+ ( -0.6737731680 * yv[0]) + (  1.6089340341 * yv[1]);
        //accXf = yv[2];
      
 //xv1[0] = xv1[1]; xv1[1] = xv1[2]; 
        //xv1[2] = accZ / GAIN;
        //yv1[0] = yv1[1]; yv1[1] = yv1[2]; 
        //yv1[2] =   (xv1[0] + xv1[2]) + 2 * xv1[1]
                     //+ ( -0.6737731680 * yv1[0]) + (  1.6089340341 * yv1[1]);
        //accZf = yv1[2];     
 //}


