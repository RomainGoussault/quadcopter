

#include <PinChangeInt.h>
#include <Servo.h>
#include <Radio.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"   
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>
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


//IMU
MPU9150Lib MPU;
bool calibrating = true;
bool IMU_problem = false;

float angles[3];


//Motors
Motors motors;
bool motors_on;
bool motors_on_old;


//FlightControl
FlightControl flightControl;
float targetAngles[3];
float throttle;


//Measuring time
unsigned long loop_time = 0;
unsigned long start_loop = 0;
float freq = 0;


//Warning LED
#define GREEN_LED_PIN 0
#define YELLOW_LED_PIN 0
#define RED_LED_PIN  0

bool green_led;
bool yellow_led;
bool red_led;


#define  SERIAL_PORT_SPEED  115200


void setup()
{
    Serial.begin( SERIAL_PORT_SPEED);
	Serial.println("Start");
	Serial.print("Loop rate: ");
	Serial.println( MPU_UPDATE_RATE);
	

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
	Serial.print("Arduino9150 starting using device ");
	Serial.println(DEVICE_TO_USE);
	Wire.begin();
	MPU.selectDevice(DEVICE_TO_USE);
	MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_ONLY	, MAG_UPDATE_RATE, MPU_LPF_RATE);// start the MPU


	//Motors
	motors.init();


	//End of the setup phase
	Serial.print("Setup done");

	
}








void loop()
{	
	// Measure loop rate
	loop_time =  micros() - start_loop;
	start_loop = micros();
	freq = 1000000/loop_time;
	//Serial.print("freq: ");
	//Serial.println(freq);
	
	
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
	//We need to wait 15 sec for the gyro autocalibration phase
	if (calibrating)		
	{
		Serial.print( "  Calibrating... ");
		if (millis() > 17000)
		{
			calibrating = false;
		}		
	}
	
	
	
	
		

	
	
	
	//Serial.println( calibrating);
	
	while (!MPU.read())  //we wait for the latest data to come
	{	delay(1);	}   
		
	if (MPU.checkValues()) //continue only the data are OK
	{
		MPU.processAngles(angles);
	//	MPU.printAngles(MPU.m_fusedEulerPose);                 // print the output of the data fusion
		Serial.println("");
	//	MPU.printProcessedAngles(angles);
	}
	else
	{
		IMU_problem = true;
		Serial.println( "    IMU PB    ");
	}
	
	
	
	//=======================================================================================
	//                                    MOTORS
	//=======================================================================================
	motors_on = RadioChannels[5] && radio_on && !IMU_problem && !calibrating;
	
	//On first start we have to make sure the thottle is down
	if (motors_on && !motors_on_old)
	{
		Serial.print("Throttle has to be down to set the motors on ");
		if (RadioChannels[1]==0)
		{
			motors_on = true;
		}
		else
		{
			motors_on = false;
		}		
	}
	motors_on_old = motors_on;
	
	
	Serial.print("ch1 ");
	Serial.println(RadioChannels[1]);
	
	Serial.print("motors_on  ");
	Serial.println(motors_on);

	if(motors_on)
	{
		targetAngles[0] =  map_f(RadioChannels[2], MAP_RADIO_LOW, MAP_RADIO_HIGH, -ROLL_MAX_RADIO, ROLL_MAX_RADIO);
		targetAngles[1] =  map_f(RadioChannels[4], MAP_RADIO_LOW, MAP_RADIO_HIGH, -PITCH_MAX_RADIO, PITCH_MAX_RADIO);
		targetAngles[2] =  map_f(RadioChannels[3], MAP_RADIO_LOW, MAP_RADIO_HIGH, -YAW_MAX_RADIO, YAW_MAX_RADIO);
		throttle = RadioChannels[1];

		flightControl.control(targetAngles, angles, throttle, motors, motors_on);

	}
	else
	{
		motors.allStop();
		//Serial.println("MOTORS STOPPED ");
	}
}






