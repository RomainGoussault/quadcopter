

#include <PinChangeInt.h>
#include <Servo.h>
#include <Radio.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h" 
#include "IMU.h" 

#include <stdlib.h>

#include <Utils.h>		
#include "Motors.h"
#include "FlightControl.h"  


#define ROLL_MAX_RADIO 10
#define PITCH_MAX_RADIO 10
#define YAW_MAX_RADIO 180





//Radio
volatile uint8_t bUpdateFlagsShared; // true if new radio signal
int RadioChannels[7];
bool radio_on;
bool ch5_old=true;


//IMU
IMU imu;
bool calibrating = true;
bool IMU_problem = false;

float angles[3];


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
float freq ;


//Warning LED
#define GREEN_LED_PIN 35
#define YELLOW_LED_PIN 31
#define RED_LED_PIN  33


bool green_led;
bool yellow_led;
bool red_led;

//Serial
#define  SERIAL_PORT_SPEED  115200

char StrMotorOn[10] = "  M  ON  ";
char StrMotorOff[10] = "  M OFF  ";
char StrMotorReady[10] = "  M Rdy  ";

	
	char Strfreq[7];
	char StrAngles[16];
	char StrSpeed[29];	
	char package[sizeof(Strfreq) + sizeof(StrSpeed) + sizeof(StrAngles) + sizeof(StrMotorOn) + 1];

	


void setup()
{
	 // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
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


	//IMU
	imu.init();
 // timer = micros();

	//Motors

	motors.init();
	
	//Warning LED
	  pinMode(GREEN_LED_PIN, OUTPUT); 
	  pinMode(RED_LED_PIN, OUTPUT);   
  	  pinMode(YELLOW_LED_PIN, OUTPUT); 
	

	//End of the setup phase
	Serial.print("Setup done");
	calibrating = false;
	
	//Printing

}








void loop()
{	
	// Measure loop rate
	loop_time =  micros() - start_loop;
	start_loop = micros();
	freq =1000000/loop_time;
    package[0] = '\0';
		//unsigned long startF = micros();
		//unsigned long endF = micros();
		//unsigned long deltaF = endF - startF;
		//Serial.print("deltaF  ");
		//Serial.print(deltaF);

	
	//=======================================================================================
	//                                   Radio
	//=======================================================================================
	// check shared update flags to see if any channels have a new signal
	if(bUpdateFlagsShared)
	{
		noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
		updateRadio();
		interrupts();
	}
	
	radio_on = getRadio(RadioChannels);

	

	//==============================================================================
	//                                   IMU
	//=======================================================================================
	if (! (imu.processAngles(angles))  ) //continue only the data are OK
		{
			IMU_problem = true;
			Serial.print( "    IMU PBPBP   ");
		}
	
	

	
	
	

	//=======================================================================================
	//                                    MOTORS
	//=======================================================================================
	motorsReady = radio_on && !IMU_problem && !calibrating; 

	if (ch5_old==false && RadioChannels[5]==true && RadioChannels[1]==0)
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
	


	
	
	//Print the status of the motor
	if (motorsReady)
	{
		if (motorsOn)
		{
			strcat(package, StrMotorOn);
		}
		else
		{
			strcat(package, StrMotorReady);
		}
	}
	else
	{
		strcat(package, StrMotorOff);
	}
	

	if(motorsReady)
	{	
		targetAngles[0] =  map_f(RadioChannels[4], MAP_RADIO_LOW, MAP_RADIO_HIGH, -ROLL_MAX_RADIO, ROLL_MAX_RADIO);
		targetAngles[1] =  map_f(RadioChannels[2], MAP_RADIO_LOW, MAP_RADIO_HIGH, -PITCH_MAX_RADIO, PITCH_MAX_RADIO);
		targetAngles[2] =  map_f(RadioChannels[3], MAP_RADIO_LOW, MAP_RADIO_HIGH, -YAW_MAX_RADIO, YAW_MAX_RADIO);
		throttle = RadioChannels[1];
		


	
		flightControl.control(targetAngles, angles, throttle, motors, motorsReady);

		
	}
	else
	{
		motors.allStop();
		//Serial.println("MOTORS STOPPED ");
	}
	
	//LEDS
	digitalWrite(GREEN_LED_PIN, motorsOn); 
	digitalWrite(YELLOW_LED_PIN, !motorsOn && motorsReady); 
	digitalWrite(RED_LED_PIN, !motorsReady); 
	
	
	
	
	//printStatus();




	dtostrf(freq,1,0,&Strfreq[2]);
	dtostrf(angles[1],6,2,&StrAngles[7]);
	dtostrf(angles[0],6,2,StrAngles);
	dtostrf(motors.getMotorSpeed(1),6,2,StrSpeed);
	dtostrf(motors.getMotorSpeed(2),6,2,&StrSpeed[7]);
	dtostrf(motors.getMotorSpeed(3),6,2,&StrSpeed[14]);
	dtostrf(motors.getMotorSpeed(4),6,2,&StrSpeed[21]);
	Strfreq[0] = 'f';
	Strfreq[1] = ' ';
	Strfreq[5] = '\t';
	Strfreq[6] = '\0';
	StrAngles[6] = '\t';
	StrAngles[13] = '\t';
	StrAngles[14] = '\t';
	StrAngles[15] = '\0';	
	StrSpeed[6] = '\t';
	StrSpeed[13] = '\t';
	StrSpeed[20] = '\t';
	StrSpeed[27] = '\t';	
	StrSpeed[28] = '\0';
	strcat(package, Strfreq);
	strcat(package, StrAngles);
	strcat(package, StrSpeed);
	strcat(package, flightControl.StrControl);

	Serial.println(package);

}



