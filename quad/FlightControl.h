/*
  FlightControl.h
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
#ifndef FlightControl_h
#define FlightControl_h


#include <Utils.h>
#include <math.h>
#include <Motors.h>
#include <Arduino.h>



//motor1: first white, motor2 second white
//4 and 2 counter clockwise

#define MAX_I_TERM 2
#define RATE_MODE 0
#define CONTROL_ON 1
#define ANGLE_LOOP_DIVIDER 7
#define  LOOP_TIME 2400


class FlightControl
{

public:

	FlightControl();
	void control(float targetAngles[], float angles[], float rates[], float throttle, Motors &motors, bool motorsReady);
	//void getLoopTime(long l);

   	//PID coefficients
	float kp_rate_roll;
	float kp_rate_pitch;
	float kp_rate_yaw;
	
	float kd_rate_roll;
	float ki_rate_roll;
	
	float kp_roll;
	float kp_pitch;
	float kp_yaw;
	
	float kd_roll;
	float kd_pitch;
	float kd_yaw;
	
	float ki_roll;
	float ki_pitch;
	float ki_yaw;
	
	
	long loop_time;
	char i_max;
	bool i_on;
 
    
    
    
private:

	
	float U1, U2, U3, U4;
	float w1, w2, w3, w4;
	
	float targetRate[3];
	
	//PID Tuning
	int incomingByte;
	float multiplier;
	
	int max_I_term;
	
	
	// Angles errors
	float anglesErrors[3];
	float anglesErrorsD[3];
	float anglesErrorsOld[3];
	float anglesErrorsSum[3];
	
	// Rates errors
	float ratesErrors[3];
	float ratesErrorsD[3];
	float ratesErrorsOld[3];
	float ratesErrorsSum[3];


	//sortie
	float sortiePIDangle[3];
	float sortiePIDrate[3];
	
	float angle_loop_time;
	int counter_angle_loop;

};

#endif
