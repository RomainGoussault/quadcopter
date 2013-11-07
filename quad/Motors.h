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
#ifndef Motors_h
#define Motors_h

#include <Arduino.h>
#include <Utils.h>

#define MOTOR_COUNT 4

#define MOTOR_1_PIN 8
#define MOTOR_2_PIN 7
#define MOTOR_3_PIN 10
#define MOTOR_4_PIN 11

#define MIN_MOTOR_SPEED_PWM 130
#define MAX_MOTOR_SPEED_PWM 230
#define MIN_MOTOR_SPEED_CONTROL 0
#define MAX_MOTOR_SPEED_CONTROL 100



//motor1: first white, motor2 second white
//1 and 3 clockwise (R on my props)
//4 and 2 counter clockwise




class Motors
{

  public:

    Motors();
    void init();
    void allStop();
    void setMotorSpeed(byte, float);
    int getMotorSpeed(byte);
    void setAllSpeed(float);
    void setMotorsOn(bool);
    



    
  private:
    int motors[MOTOR_COUNT];
    float motor_speeds[MOTOR_COUNT];
    bool motorsOn;


    

};

#endif
