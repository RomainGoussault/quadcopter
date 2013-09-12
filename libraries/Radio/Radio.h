/*
  Radio.h - Library 
*/

#ifndef Radio_h
#define Radio_h

#include "Arduino.h"

#define  PRINT_ALL_CHANNELS 0
#define  RADIO_POWER_PIN   51

#define CH1_IN_PIN A12
#define CH2_IN_PIN A14
#define CH3_IN_PIN A15
#define CH4_IN_PIN A10
#define CH5_IN_PIN A13
#define CH6_IN_PIN A11

#define CH1_FLAG 1
#define CH2_FLAG 2
#define CH3_FLAG 4
#define CH4_FLAG 8
#define CH5_FLAG 16
#define CH6_FLAG 32

#define MAX_1 2048
#define MAX_2 1984
#define MAX_3 1940
#define MAX_4 1960
#define MAX_5 2032
#define MAX_6 2032

#define MIN_1 968
#define MIN_2 952
#define MIN_3 869
#define MIN_4 880//908
#define MIN_5 936
#define MIN_6 936

#define MID_1 (MAX_1 + MIN_1)/2
#define MID_2 (MAX_2 + MIN_2)/2
#define MID_3 (MAX_3 + MIN_3)/2
#define MID_4 (MAX_4 + MIN_4)/2
#define MID_5 (MAX_5 + MIN_5)/2
#define MID_6 (MAX_6 + MIN_6)/2

#define MAP_RADIO_HIGH 1000
#define MAP_RADIO_LOW 0
#define MAP_RADIO_MID (MAP_RADIO_HIGH + MAP_RADIO_LOW)/2

#define MAP_RADIO_HIGH 1000

#define DEADZONE_ENABLE 1
#define DEADZONE_PERCENT 0.05

#define DEADZONE (MAP_RADIO_HIGH - MAP_RADIO_LOW ) * DEADZONE_PERCENT




void updateRadio();
void getRadio(int pChannels[]);


void calcCh1();
void calcCh2();
void calcCh3();
void calcCh4();
void calcCh5();
void calcCh6();


#endif

