/*
  Melody
 
 Plays a melody
 
 circuit:
 * 8-ohm speaker on digital pin 8
 100 ohm resistor (marron vert marron)
 
 created 21 Jan 2010
 modified 30 Aug 2011
 by Tom Igoe

This example code is in the public domain.
 
 http://arduino.cc/en/Tutorial/Tone
 
 */



void setup() {
 
}

void loop() {
  // no need to repeat the melody.
   tone(8, 33,1000/4);
}
