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
	float ku = 0.065/100;
	kp_roll= 0.001;
	kd_roll = 0;//0.075*ku*1.1 / 0.02;
	ki_roll = 0*0.5*ku*0.9*0.02;
	anglesErrorsSum[0]=0;
}




void FlightControl::control(float targetAngles[], float angles[], float throttle, Motors &motors, bool motorsReady) {
	
	//omega = 1.0/200.0*(millis()-17000) /1000;
	int incomingByte = 0;
	float targetRate[3];
	float multiplier = 1.1;
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
			kd_roll *= multiplier;
			ki_roll *= multiplier;
		}
		if (incomingByte == 'p' )
		{
			Serial.print("Nouvelle valeur de kp_roll ");
			kp_roll /= multiplier;
			kd_roll /= multiplier;
			ki_roll /= multiplier;
		}
	}
	

	
	for (int i = 0; i < 3 ; i++)
	{

		anglesErrors[i] = targetAngles[i] - angles[i];
		anglesErrorsD[i] = ( anglesErrors[i] - anglesErrorsOld[i]) ;
		anglesErrorsSum[i] += anglesErrors[i];
		constrain(anglesErrorsSum[i], -MAX_I_TERM, MAX_I_TERM);
		anglesErrorsOld[i] = anglesErrors[i];
	}
	

	
	//Roll is control by M2 and M4
	//Ptich is control by M1 and M3

	float U1, U2, U3, U4;
	float w1, w2, w3, w4;


	U2 = (kp_roll * anglesErrors[0]);
	//U2 = 0.2*sin(omega*millis()/1000);
	U3 = -0*(kp_pitch * anglesErrors[1] );
	U4 =0*(kp_yaw * anglesErrors[2])	 ;

	U1 = map_f(throttle, MAP_RADIO_LOW , MAP_RADIO_HIGH, 0, 80);


	w1 =  0*(U3+U1-U4);
	w4 = 1* (- U2+U4+U1);
	w3 =  0*(U1- U3-U4);
	w2= 1*( U4+ U2+U1);
	//swith 2 et 4





	if (w1<0) {
		w1=0;
	} else {
	//	w1=sqrt(w1);
	}

	if (w2<0) {
		w2=0;
	} else {
	//	w2=sqrt(w2);
	}

	if (w3<0) {
		w3=0;
	} else {
	//	w3=sqrt(w3);
	}

	if (w4<0) {
		w4=0;
	} else {
	//w4=sqrt(w4);
	}


	StrControl[0]='\t';
	StrControl[1]='k';
	StrControl[2]='p';
	StrControl[3]=' ';
	dtostrf(kp_roll,6,4,&StrControl[4]);
	StrControl[10]='\0';	
	
	
	

	//Serial.print(" w1 ");
	//Serial.print(w1);

	//Serial.print("|w2 ");
	//Serial.print(w2);

	//Serial.print("|w3 ");
	//Serial.print(w3);

	//Serial.print("|w4 ");
	//Serial.print(w4);
	//Serial.print("|");

	motors.setMotorSpeed(1, w1);
	motors.setMotorSpeed(2, w2);
	motors.setMotorSpeed(3, w3);
	motors.setMotorSpeed(4, w4);
	

	//Serial.print("   U2 ");
	//Serial.print(U2,4);	
	
	//Serial.print("   omega  ");
	//Serial.print(omega,4);		


	//Serial.print("\t kp");
	//Serial.print(kp_roll,4);	
	
	//Serial.print("   kd roll ");
	//Serial.print(kd_roll,4);
	
	//Serial.print("   ki roll ");
	//Serial.print(ki_roll,4);	
	
	
}



