/*
  Utils.cpp - Utility functions for my quadcopter code


  Based on the code of Myles Grant <myles@mylesgrant.com>
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

#include "Arduino.h"

float map_f(float  x, float  in_min,float   in_max, float  out_min, float  out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//new version of map_f which is a bit faster for the RADIO data than map_f
float map_f_s(int  x, int  in_min, int out_min, int ratio)
{
	return (x - in_min) / ratio + out_min;
}

float mean(int m, float a[]) {
    int sum=0, i;
    for(i=0; i<m; i++)
        sum+=a[i];
    return((float)sum/m);
}

// Insert sort. From "whistler" - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1283456170/0
void isort(int *a, byte n){
	for (int i = 1; i < n; ++i){
		int j = a[i];
		int k;
		for (k = i - 1; (k >= 0) && (j < a[k]); k--){
			a[k + 1] = a[k];
		}
    a[k + 1] = j;
	}
}

int findMedian(int *data, byte arraySize){
	isort(data, arraySize);
	return data[arraySize/2];
}

// Low pass filter, kept as regular C function for speed
float filterSmooth(float currentData, float previousData, float smoothFactor){
	if (smoothFactor != 1.0) //only apply time compensated filter if smoothFactor is applied
		return (previousData * (1.0 - smoothFactor) + (currentData * smoothFactor)); 
	else
		return currentData; //if smoothFactor == 1.0, do not calculate, just bypass!
}

int8_t sgn(int val) {
	if (val < 0) return -1;
	if (val==0) return 0;
	return 1;
}

float constrain_f(float x, float min, float max)
{
	if (x>max)
	{
		return max;
	}
	if (x<min)
	{
		return min;
	}
	else
	{
		return x;
	}
}