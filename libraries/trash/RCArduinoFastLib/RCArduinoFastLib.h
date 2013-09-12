/*****************************************************************************************************************************/
// RCArduinoFastLib by DuaneB is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
//
// http://rcarduino.blogspot.com
//
/*****************************************************************************************************************************/

#include "Arduino.h"

// COMMENT OR UNCOMMENT THIS LINE TO ENABLE THE SECOND BANK OF SERVOS
//#define MORE_SERVOS_PLEASE 1

// the first bank of servos uses OC1A - this will disable PWM on digital pin 9 - a small price for 10 fast and smooth servos
// the second bank of servos uses OC1B - this will disable PWM on digital pin 10 - a small price for 10 more fast and smooth servos

// The library blindly pulses all ten servos one and after another
// If you change the RC_CHANNEL_OUT_COUNT to 4 servos, the library will pulse them more frequently than
// it can ten -
// 10 servos at 1500us = 15ms = 66Hz
// 4 Servos at 1500us = 6ms = 166Hz
// if you wanted to go even higher, run two servos on each timer
// 2 Servos at 1500us = 3ms = 333Hz
//
// You might not want a high refresh rate though, so the setFrameSpace function is provided for you to
// add a pause before the library begins its next run through the servos
// for 50 hz, the pause should be to (20,000 - (RC_CHANNEL_OUT_COUNT * 2000))

// Change to set the number of servos/ESCs
#define RC_CHANNEL_OUT_COUNT 4

#if defined (MORE_SERVOS_PLEASE)
#define RCARDUINO_MAX_SERVOS (RC_CHANNEL_OUT_COUNT*2)
#else
#define RCARDUINO_MAX_SERVOS (RC_CHANNEL_OUT_COUNT)
#endif

// Minimum and Maximum servo pulse widths, you could change these,
// Check the servo library and use that range if you prefer
#define RCARDUINO_SERIAL_SERVO_MIN 1000
#define RCARDUINO_SERIAL_SERVO_MAX 2000
#define RCARDUINO_SERIAL_SERVO_DEFAULT 1500

#define RC_CHANNELS_NOPORT 0
#define RC_CHANNELS_PORTB 1
#define RC_CHANNELS_PORTC 2
#define RC_CHANNELS_PORTD 3
#define RC_CHANNELS_NOPIN 255

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CRCArduinoFastServos
//
// A class for generating signals in combination with a 4017 Counter
//
// Output upto 10 Servo channels using just digital pins 9 and 12
// 9 generates the clock signal and must be connected to the clock pin of the 4017
// 12 generates the reset pulse and must be connected to the master reset pin of the 4017
//
// The class uses Timer1, as this prevents use with the servo library
// The class uses pins 9 and 12
// The class does not adjust the servo frame to account for variations in pulse width,
// on the basis that many RC transmitters and receivers designed specifically to operate with servos
// output signals between 50 and 100hz, this is the same range as the library
//
// Use of an additional pin would provide for error detection, however using pin 12 to pulse master reset
// at the end of every frame means that the system is essentially self correcting
//
// Note
// This is a simplified derivative of the Arduino Servo Library created by Michael Margolis
// The simplification has been possible by moving some of the flexibility provided by the Servo library
// from software to hardware.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////


class CRCArduinoFastServos
{
  public:
    static void setup();

    // configures timer1
    static void begin();

    // called by the timer interrupt service routine, see the cpp file for details.
    static void OCR1A_ISR();

#if defined(MORE_SERVOS_PLEASE)
    static void OCR1B_ISR();
#endif

    // called to set the pulse width for a specific channel, pulse widths are in microseconds - degrees are for wimps !
    static void attach(uint8_t nChannel,uint8_t nPin);
    static void writeMicroseconds(uint8_t nChannel,uint16_t nMicroseconds);
    static void setFrameSpaceA(uint8_t sChannel,uint16_t unMicroseconds);
    static void setFrameSpaceB(uint8_t sChannel,uint16_t unMicroseconds);

  protected:
    class CPortPin
    {
      public:
        //uint8_t m_sPort;
        volatile unsigned char *m_pPort;
        uint8_t m_sPinMask;
        uint16_t m_unPulseWidth;
    };

    // this sets the value of the timer1 output compare register to a point in the future
    // based on the required pulse with for the current servo
    static void setOutputTimerForPulseDurationA() __attribute__((always_inline));


    static void setChannelPinLowA(uint8_t sChannel) __attribute__((always_inline));
    static void setCurrentChannelPinHighA();

    // Easy to optimise this, but lets keep it readable instead, its short enough.
    static volatile uint8_t*  getPortFromPin(uint8_t sPin) __attribute__((always_inline));
    static uint8_t getPortPinMaskFromPin(uint8_t sPin) __attribute__((always_inline));

    // Records the current output channel values in timer ticks
    // Manually set by calling writeChannel, the function adjusts from
    // user supplied micro seconds to timer ticks
    volatile static CPortPin m_ChannelOutA[RC_CHANNEL_OUT_COUNT];
    // current output channel, used by the timer ISR to track which channel is being generated
    static uint8_t m_sCurrentOutputChannelA;

#if defined(MORE_SERVOS_PLEASE)
    // Optional channel B for servo number 10 to 19
    volatile static CPortPin m_ChannelOutB[RC_CHANNEL_OUT_COUNT];
    static uint8_t m_sCurrentOutputChannelB;
    static void setOutputTimerForPulseDurationB();

    static void setChannelPinLowB(uint8_t sChannel) __attribute__((always_inline));
    static void setCurrentChannelPinHighB() __attribute__((always_inline));
#endif

    // two helper functions to convert between timer values and microseconds
    static uint16_t ticksToMicroseconds(uint16_t unTicks) __attribute__((always_inline));
    static uint16_t microsecondsToTicks(uint16_t unMicroseconds) __attribute__((always_inline));
};

// Change to set the number of channels in PPM Input stream
#define RC_CHANNEL_IN_COUNT 3
// two ticks per us, 3000 us * 2 ticks = 6000 minimum frame space
#define MINIMUM_FRAME_SPACE 6000
#define MAXIMUM_PULSE_SPACE 5000

class CRCArduinoPPMChannels
{
  public:
    static void begin();
    static void INT0ISR();
    static uint16_t getChannel(uint8_t nChannel);
    static uint8_t getSynchErrorCounter();

    protected:
    static void forceResynch();

    static volatile uint16_t m_unChannelSignalIn[RC_CHANNEL_IN_COUNT];
    static uint8_t m_sCurrentInputChannel;

    static uint16_t m_unChannelRiseTime;
    static volatile uint8_t m_sOutOfSynchErrorCounter;
};
