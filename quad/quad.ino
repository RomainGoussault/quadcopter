/*
  Quad.ino - Code to control a quadcopter.
  Created by Romain Goussault <romain.goussault@gmail.com>
  
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


#include <PinChangeInt.h>
#include <Servo.h>
#include <Radio.h>
//#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h" 
#include "IMU.h" 
#include <stdlib.h>
#include <Utils.h>		
#include "Motors.h"
#include "FlightControl.h"  

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Define max input angles
#define ROLL_MAX_RADIO 15
#define PITCH_MAX_RADIO 15
#define YAW_MAX_RADIO 180

#define GREEN_LED_PIN 35
#define YELLOW_LED_PIN 31
#define RED_LED_PIN  33

//Serial speed
#define  SERIAL_PORT_SPEED  115200




//Radio
volatile uint8_t bUpdateFlagsShared; // true if new radio signal
int RadioChannels[7];
bool radio_on;
bool ch5_old=true;

const int roll_map_ratio = 1/ (ROLL_MAX_RADIO - (-ROLL_MAX_RADIO)) / (MAP_RADIO_HIGH - (MAP_RADIO_LOW));
const int pitch_map_ratio =1/  (PITCH_MAX_RADIO - (-PITCH_MAX_RADIO)) / (MAP_RADIO_HIGH - (MAP_RADIO_LOW));
const int yaw_map_ratio =1/ (YAW_MAX_RADIO - (-YAW_MAX_RADIO)) / (MAP_RADIO_HIGH - (MAP_RADIO_LOW));


//IMU
IMU imu;
bool calibrating = true;
bool IMU_problem = false;
float max_X ;
float max_Y ;
float angles[3];
float rates[3];
float old_a;
float mdiff;


//Motors
Motors motors;
bool motorsReady = false;
bool motorsReadyOld = false;
bool motorsOn = false;



//FlightControl
FlightControl flightControl;
float targetAngles[3];
float throttle;



//Measuring time
unsigned long loop_time = 0;
unsigned long start_loop = 0;
int freq;
double deltaF;



//Warning LED
bool green_led;
bool yellow_led;
bool red_led;



//Printing
char StrMotor[4];
char StrMotorOn[4] = "ON";
char StrMotorOff[4] = "OFF";
char StrMotorReady[4] = "Rdy";
char StrControl[6];
char Strfreq[7];
char StrAngles[6];
char StrSpeed[4];	

int print_counter = 0;

const int frequency_print_offset = 4;
const int angle_print_offset =  frequency_print_offset+7;
const int motor_print_offset = angle_print_offset+17;
const int control_print_offset = motor_print_offset+7;



void setup()
{
	//Warning LED
	pinMode(GREEN_LED_PIN, OUTPUT); 
	pinMode(RED_LED_PIN, OUTPUT);   
  	pinMode(YELLOW_LED_PIN, OUTPUT); 
  	
	digitalWrite(GREEN_LED_PIN, motorsOn); 
	digitalWrite(YELLOW_LED_PIN, !motorsOn && motorsReady); 
	digitalWrite(RED_LED_PIN, !motorsReady); 
  	  
  	  
	 // join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
       Fastwire::setup(800	, true); //800kHz is the maximum rate I have achived on the Arduino MEGA
    #endif
    
    Serial.begin( SERIAL_PORT_SPEED);
	Serial.println("Start");


	//RADIO
	//Power pin
	pinMode(RADIO_POWER_PIN , OUTPUT);
	digitalWrite(RADIO_POWER_PIN , HIGH);

	// Using the PinChangeInt library, attach the interrupts to read radio signal
	PCintPort::attachInterrupt(CH1_IN_PIN, calcCh1,CHANGE);
	PCintPort::attachInterrupt(CH2_IN_PIN, calcCh2,CHANGE);
	PCintPort::attachInterrupt(CH3_IN_PIN, calcCh3,CHANGE);
	PCintPort::attachInterrupt(CH4_IN_PIN, calcCh4,CHANGE);
	PCintPort::attachInterrupt(CH5_IN_PIN, calcCh5,CHANGE);
	PCintPort::attachInterrupt(CH6_IN_PIN, calcCh6,CHANGE);


	imu.init();
	motors.init();
	
	
	//End of the setup phase
	Serial.print("Setup done");
	calibrating = false;
}





void loop()
{	
	// Measure loop rate
	start_loop = micros();
	freq =1000000/loop_time;


	//======================================================================================
	//                                   Radio
	//======================================================================================
	// check shared update flags to see if any channels have a new signal
	if(bUpdateFlagsShared)
	{
		noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
		updateRadio();
		interrupts();
	}
	radio_on = getRadio(RadioChannels);


	//======================================================================================
	//                                   IMU
	//======================================================================================
	if (! (imu.processAngles(angles, rates))  ) //continue only the data from the IMU is acceptable
		{
			IMU_problem = true;
			Serial.print( "    IMU PROBLEM   ");
		}


	//=======================================================================================
	//                                    MOTORS
	//=======================================================================================
	motorsReady = radio_on && !IMU_problem && !calibrating; 

	if (ch5_old==false && RadioChannels[5]==true && RadioChannels[1]==0 ) 
	{
		motorsOn = true;
	}
	if (RadioChannels[5]==false || (motorsReadyOld==true  &&  motorsReady==false))
	{
		motorsOn = false;
	}
		
	motors.setMotorsOn(motorsOn);
	ch5_old=RadioChannels[5];
	motorsReadyOld = motorsReady;

	if(motorsReady)
	{	
		//The following expressions replace the map expression. They are much faster at run-time
		  targetAngles[0] = RadioChannels[4]*0.03-15;
          targetAngles[1] = RadioChannels[2]*0.03-15;
          targetAngles[2] = RadioChannels[3]*0.36-180;
		
		throttle = RadioChannels[1];
		
		flightControl.control(targetAngles, angles, rates, throttle, motors, motorsReady);
	}
	else
	{
		motors.allStop();
	}
	
	//LEDS
	digitalWrite(GREEN_LED_PIN, motorsOn); 
	digitalWrite(YELLOW_LED_PIN, !motorsOn && motorsReady); 
	digitalWrite(RED_LED_PIN, !motorsReady); 
	
	
	/*
	 * Printing info to the serial 
	 * Info includes: frequency of the main loop, angles from the IMU, motors commands and PID coeff
	 * At each loop only a fraction of the whole "quadcopter status" is printed to save time
	*/
	switch (print_counter)
	  {

		//Frequency print
	  case 1: 
		   Serial.print("f ");  //64 µs
		   break;
	  case 2:
		   itoa(freq,Strfreq,10);
		   break;
	  case 3:
		   Serial.print(Strfreq);  //
		   break;
	  case 4:
		   Serial.print("    ");  
		   break;
 		    
		//Angles print
	  case frequency_print_offset+1: //
		   dtostrf(angles[0],6,2,StrAngles);
		   break;
	  case frequency_print_offset+2:
		   Serial.print(StrAngles);  //
		   break;
	  case frequency_print_offset+3:
		   Serial.print(" ");  //
		   break;
	  case frequency_print_offset+4:
		   dtostrf(angles[1],6,2,StrAngles);
		   break;
	  case frequency_print_offset+5:
		   Serial.print(" ");  //
		   break;		   
	  case frequency_print_offset+6:
		   Serial.print(StrAngles);  //
		   break;		   
	  case frequency_print_offset+7://11
		   Serial.print("    ");  //
		   break;		      
		   
		//Motors print
	  case angle_print_offset+1: //13
			if (motorsReady)
			{
				if (motorsOn)
				{
					strncpy(StrMotor, StrMotorOn, sizeof(StrMotor) - 1);
				}
				else
				{
					strncpy(StrMotor, StrMotorReady, sizeof(StrMotor) - 1);
				}
			}
			else
			{
				strncpy(StrMotor, StrMotorOff, sizeof(StrMotor) - 1);
			}
			break;			
	  case angle_print_offset+2:
		   Serial.print(StrMotor);  //
		   break;
	  case angle_print_offset+3:
		   Serial.print(" ");  //
		   break;
	  case angle_print_offset+4:
		   Serial.print(" ");  //
		   break;		   
	  case angle_print_offset+5: 
		   itoa(motors.getMotorSpeed(1),StrSpeed,10);
		   break;
	  case angle_print_offset+6:
		   Serial.print(StrSpeed);  //
		   break;
	  case angle_print_offset+7:
		   Serial.print("  ");  //
		   break;			   
	  case angle_print_offset+8:
		   itoa(motors.getMotorSpeed(2),StrSpeed,10);
		   break;
	  case angle_print_offset+9:
		   Serial.print(StrSpeed);  //
		   break;	
	  case angle_print_offset+10:  //23
		   Serial.print("  ");  //
		   break;			   
	  case angle_print_offset+11: 
		   itoa(motors.getMotorSpeed(3),StrSpeed,10);
		   break;
	  case angle_print_offset+12:
		   Serial.print(StrSpeed);  //
		   break;
	  case angle_print_offset+13:
		   Serial.print("  ");  //
		   break;
	  case angle_print_offset+14:
		   itoa(motors.getMotorSpeed(4),StrSpeed,10);
		   break;		   
	  case angle_print_offset+15:
		   Serial.print(StrSpeed);  //
		   break;			   	   
	  case angle_print_offset+16:
		   Serial.print("    ");  //
		   break;		  
	  case angle_print_offset+17:
		   Serial.print("    ");  //
		   break;			   
		   
		   //Control print
	  case motor_print_offset+1:
		   Serial.print("kp ");  //
		   break;				   
	  case motor_print_offset+2:
			dtostrf(flightControl.kp_roll,6,2,StrControl);
		   break;	
	  case motor_print_offset+3:
		   Serial.print(StrControl);  //
		   break;				   
	  case motor_print_offset+4:
	    	Serial.print("  ");  //
		   break;		   
	    case motor_print_offset+5:
			dtostrf(flightControl.kd_roll,6,2,StrControl);
		   break;	
	  case motor_print_offset+6:
		   Serial.print(StrControl);  //
		   break;				   
	  case motor_print_offset+7:
	    	Serial.print("  ");  //
		   break;		   
		   case motor_print_offset+8:
			dtostrf(flightControl.i_on,1,2,StrControl);
		   break;		   
	  case motor_print_offset+9:
		   Serial.print(StrControl);  //
		   break;				   
	  case motor_print_offset+10:
	    	Serial.print("  ");  //
		   break;		
	  case control_print_offset+11:
		   Serial.println("");  //
		   print_counter=1;
		   break;
	  }
	 print_counter++;
	 
	 
	 

	while ((micros() - start_loop)<LOOP_TIME)
	{
		
	}
	
	loop_time =  micros() - start_loop; //Calculating loop_time to calculate frequency
}



