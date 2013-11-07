/*
  Utils.h - Utility functions for my quadcopter code
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
#ifndef Utils_h
#define Utils_h

#include "Arduino.h"

#define rac22 0.7071



float constrain_f(float x, float min, float max);
float map_f(float  x, float  in_min, float in_max, float out_min, float out_max);
float map_f_s(int  x, int  in_min, int out_min, int ratio);
float mean(int , float[]);
void isort(int *, byte);
int findMedian(int *, byte);
float filterSmooth(float, float, float);
int8_t sgn(int);
void printStatus();



#endif
