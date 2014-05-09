/*
  FlightControl.cpp
  Created by Romain Goussault <romain.goussault@gmail.com>

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
	
	//Setting up the gains
	float ku = 3.0/100;
	kp_roll= 0.5*ku;
	ki_roll = 0.001*1.2*ku*2/angle_loop_time*1000000;
	kd_roll = 1000*0.075*ku*0.5*angle_loop_time/1000000;

	kp_rate_roll = 0.2;
	
	// I-term init
	anglesErrorsSum[0]=0;
	i_on = false;
	i_max='0';
	
	//incomingByte is used to tune the PID gains through Serial
	incomingByte = 0;
	multiplier = 1.1;	

	//the attitude (angle) loop runs ANGLE_LOOP_DIVIDER times slower than the speed loop
	angle_loop_time = LOOP_TIME* ANGLE_LOOP_DIVIDER;
	counter_angle_loop=ANGLE_LOOP_DIVIDER;
}




void FlightControl::control(float targetAngles[], float angles[], float rates[], float throttle, Motors &motors, bool motorsReady) {
		
	//Setting PID gains
	if (Serial.available() > 0) 
	{ 
		incomingByte = Serial.read();

		if (incomingByte == 'P' )
		{
			Serial.print("Increasing k_roll");
			kp_roll *= multiplier;
			kd_roll *= multiplier;
			ki_roll *= multiplier;
			
			//kp_rate_roll *= multiplier;
			////ki_rate_roll *= multiplier;	
			////kd_rate_roll *= multiplier;		
		}
		if (incomingByte == 'p' )
		{
			Serial.print("Decreasing k_roll");
			kp_roll /= multiplier;
			kd_roll /= multiplier;
			ki_roll /= multiplier;
			
			//kp_rate_roll /= multiplier;
			//ki_rate_roll /= multiplier;
			//kd_rate_roll /= multiplier;
		}
		
		if (incomingByte == 'i' )
		{
			i_on=!i_on;
		}
		if (incomingByte == 'D' )
		{
			kd_roll *= multiplier;
		}
		if (incomingByte == 'd' )
		{
			kd_roll /= multiplier;
		}
		
		if (incomingByte == 'R' )
		{
			kp_rate_roll *= multiplier;
		}
		if (incomingByte == 'r' )
		{
			kp_rate_roll /= multiplier;
		}
	}
	

	if (RATE_MODE)
	{
		//Speed Loop
		//Only a Proportionnal feedback for now
		for (int i = 0; i < 2 ; i++)
		{
			//targetRate[i] = targetAngles[i];
			//ratesErrors[i] = targetRate[i] - rates[i];
			ratesErrors[i] = targetAngles[i] - rates[i];
			//ratesErrorsD[i] = kd_rate_roll*( ratesErrors[i] - ratesErrorsOld[i]) ;	
			//ratesErrorsSum[i] += i_on*ki_rate_roll*ratesErrors[i];
			//ratesErrorsSum[i]= i_on*constrain_f( anglesErrorsSum[i], -MAX_I_TERM, MAX_I_TERM);
			//ratesErrorsOld[i] = ratesErrors[i];
			sortiePIDrate[i] = kp_rate_roll * ratesErrors[i] ;//+  ratesErrorsSum[i] + ratesErrorsD[i];
		}
		U2 = 1*CONTROL_ON*kp_rate_roll * ratesErrors[0] ;
		U3 = 1*CONTROL_ON*kp_rate_roll * ratesErrors[1] ;
	}
	else
	{
		//Attitude (Angle) Loop
		if (counter_angle_loop==ANGLE_LOOP_DIVIDER)
		{
			for (int i = 0; i < 2 ; i++)
			{
				anglesErrors[i] = targetAngles[i] - angles[i];
				anglesErrorsD[i] = kd_roll*( anglesErrors[i] - anglesErrorsOld[i]) ;
				anglesErrorsSum[i] += i_on*ki_roll*anglesErrors[i];
				anglesErrorsSum[i]= i_on*constrain_f( anglesErrorsSum[i], -MAX_I_TERM, MAX_I_TERM);
				anglesErrorsOld[i] = anglesErrors[i];
				sortiePIDangle[i] = kp_roll * anglesErrors[i] + anglesErrorsSum[i] + anglesErrorsD[i];	
			}
			counter_angle_loop=0;
		}
		
		//Speed Loop 
		for (int i = 0; i < 2 ; i++)
		{
			targetRate[i] = sortiePIDangle[i];
			ratesErrors[i] = targetRate[i] - rates[i];
		}

		U2 = 1*CONTROL_ON*(kp_rate_roll * ratesErrors[0]);
		U3 = 1*CONTROL_ON*(kp_rate_roll * ratesErrors[1]);
	}
	

	//U1 = map_f(throttle, MAP_RADIO_LOW , MAP_RADIO_HIGH, 0, 80);
	U1 = throttle*0.10;
	U4=CONTROL_ON*0; // TODO: Implement Yaw control
		
	//Roll is control by M2 and M4 via U2
	//Pitch is control by M1 and M3 via U3
	w2=  1* (U1 + U2 + U4);  // 
	w1 = 1* (U1 + U3 - U4); //
	w4 = 1* (U1 - U2 + U4); //
	w3 = 1* (U1 - U3 - U4);  //

	if (w1<0) {
		w1=0;} 

	if (w2<0) {
		w2=0;}

	if (w3<0) {
		w3=0;}

	if (w4<0) {
		w4=0;} 

	if (  i_on)
	{
		i_max='1';
	}
	else
	{
		i_max='0';
	}

	motors.setMotorSpeed(1, 1*w1);
	motors.setMotorSpeed(2, 1*w2);
	motors.setMotorSpeed(3, 1*w3);
	motors.setMotorSpeed(4, 1*w4);

	counter_angle_loop++;
}



