/*
  FlightControl.cpp

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License  for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include "FlightControl.h"
#include "Radio.h"



FlightControl::FlightControl() {
	kp_roll= 0.0001;
}

void FlightControl::init() {
	
}



void FlightControl::control(float targetAngles[], float angles[], float throttle, Motors motors, bool motorsReady) {
	
	int incomingByte = 0;
	float targetRate[3];
	
	float multiplier = 1.05;
	//kp_pitch= *1;
	//kp_yaw=0.1;
 
	//kd_roll=multiplier *1;
	//kd_pitch=multiplier *1;
	//kd_yaw=0.1; 
	
	//ki_roll=0;
	//ki_pitch=0;
	//ki_yaw=0.1;
	
	
	
	//Setting gain of the PID
	if (Serial.available() > 0) 
	{ 
		incomingByte = Serial.read();

		if (incomingByte == 'P' )
		{
			Serial.print("Nouvelle valeur de kp_roll ");
			kp_roll *= multiplier;
		}
		if (incomingByte == 'p' )
		{
			Serial.print("Nouvelle valeur de kp_roll ");
			kp_roll /= multiplier;
		}
	}
	

	
	for (int i = 0; i < 3 ; i++)
	{
		anglesErrors[i] = targetAngles[i] - angles[i];
		anglesOld[i] = angles[i];
	}
	

	
	//Roll is control by M2 and M4
	//Ptich is control by M1 and M3

	float U1, U2, U3, U4;
	float w1, w2, w3, w4;


	U2 = (kp_roll * anglesErrors[0] );
	U3 = -0*(kp_pitch * anglesErrors[1] );
	U4 =0*(kp_yaw * anglesErrors[2])	 ;

	U1 = map_f(throttle, MAP_RADIO_LOW , MAP_RADIO_HIGH, 0, 2*100);


	w1 =0*(0.5*U3+0.25*U1-0.25*U4);
	w4 = - 0.5*U2+0.25*U4+0.25*U1;
	w3 = 0*(0.25*U1- 0.5*U3-0.25*U4);
	w2=    0.25*U4+ 0.5*U2+0.25*U1;
	//swith 2 et 4





	if (w1<0) {
		w1=0;
	} else {
		w1=sqrt(w1);
	}

	if (w2<0) {
		w2=0;
	} else {
		w2=sqrt(w2);
	}

	if (w3<0) {
		w3=0;
	} else {
		w3=sqrt(w3);
	}

	if (w4<0) {
		w4=0;
	} else {
		w4=sqrt(w4);
	}





	Serial.print(" w1: ");
	Serial.print(w1);

	Serial.print("| w2: ");
	Serial.print(w2);

	Serial.print("| w3: ");
	Serial.print(w3);

	Serial.print("| w4: ");
	Serial.print(w4);
	Serial.print("|    ");

	motors.setMotorSpeed(1, w1);
	motors.setMotorSpeed(2, w2);
	motors.setMotorSpeed(3, w3);
	motors.setMotorSpeed(4, w4);

	Serial.print("   kp roll ");
	Serial.print(kp_roll,6);	
}



