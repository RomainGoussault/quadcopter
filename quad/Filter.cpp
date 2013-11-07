

#include "Filter.h"




Filter::Filter(){  
inv_gain = 1/GAIN;
}


float Filter::update(float input){
	
       //xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; xv[5] = xv[6]; 
        //xv[6] =  input  *GAIN;
        //yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; yv[5] = yv[6]; 
        //yv[6] =   (xv[0] + xv[6]) + 6 * (xv[1] + xv[5]) + 15 * (xv[2] + xv[4])
                     //+ 20 * xv[3]
                     //+ ( -0.3384356188 * yv[0]) + (  2.3955407240 * yv[1])
                     //+ ( -7.1030805311 * yv[2]) + ( 11.2986621500 * yv[3])
                     //+ (-10.1746976070 * yv[4]) + (  4.9217239814 * yv[5]);
    //return yv[6];
      xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; 
        xv[5] =  input  * inv_gain;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; 
        yv[5] =   (xv[0] + xv[5]) + 5 * (xv[1] + xv[4]) + 10 * (xv[2] + xv[3])
                     + (  0.1435712906 * yv[0]) + ( -1.0082242502 * yv[1])
                     + (  2.8918039062 * yv[2]) + ( -4.2471464996 * yv[3])
                     + (  3.2060294246 * yv[4]);
        return yv[5];
     
	
	}




