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

#include <Arduino.h>
#include <Utils.h>

#define  ROLL_MAX_RADIO 20*DEGREE_TO_RAD  
#define  PITCH_MAX_RADIO 20*DEGREE_TO_RAD  
#define  YAW_MAX_RADIO 150*DEGREE_TO_RAD   

  

//motor1: first white, motor2 second white
//4 and 2 counter clockwise




class FlightControl
{

  public:

    FlightControl();
    void init();
    void control(float, float, float, float, float, float, float);
float e_roll;
float e_pitch;
float e_yaw;


    
  private:


    

};

#endif
