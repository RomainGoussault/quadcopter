#define MAX_1 2669
#define MAX_2 2614
#define MAX_3 2540
#define MAX_4 2542
#define MAX_5 2668
#define MAX_6 2662

#define MIN_1 1257
#define MIN_2 1251
#define MIN_3 1138
#define MIN_4 1159
#define MIN_5 1217
#define MIN_6 1217

#define MAP_RADIO_HIGH 100
#define MAP_RADIO_LOW 0


unsigned long ch1=0;
unsigned long ch2=0;
unsigned long ch3=0;
unsigned long ch4=0;
unsigned long ch5=0;
unsigned long ch6=0;
unsigned long ch7=0;
unsigned long max2=0;



void setup()
{

// Digital pins for the radio  
pinMode (8, INPUT);
pinMode (9, INPUT);
pinMode (10, INPUT);
pinMode (11, INPUT);
pinMode (12, INPUT);
pinMode (13, INPUT);


Serial.begin(57600);
Serial.println("Done with setup");
}

void checkTransmitter()
{

// get the values  
ch4  = pulseIn (8, HIGH, 100000); //read RC channel, wait max of 0.1 seconds for pulse
ch6 = pulseIn (9, HIGH, 100000); //read RC channel, wait max of 0.1 seconds for pulse
ch1  = pulseIn (10, HIGH, 100000); //read RC channel, wait max of 0.1 seconds for pulse
ch5  = pulseIn (11, HIGH, 100000); //read RC channel, wait max of 0.1 seconds for pulse
ch2   = pulseIn (12, HIGH, 100000); //read RC channel, wait max of 0.1 seconds for pulse
ch3  = pulseIn (13, HIGH, 100000); //read RC channel, wait max of 0.1 seconds for pulse
max2 = max(ch2,max2);

//value in [MAP_RADIO_LOW - MAP_RADIO_HIGH]
ch1 = MAP_RADIO_HIGH - map(ch1, MIN_1, MAX_1, MAP_RADIO_LOW, MAP_RADIO_HIGH);
ch2 = map(ch2, MIN_2, MAX_2, MAP_RADIO_LOW, MAP_RADIO_HIGH);
ch3 = map(ch3, MIN_3, MAX_4, MAP_RADIO_LOW, MAP_RADIO_HIGH);
ch4 = MAP_RADIO_HIGH - map(ch4, MIN_4, MAX_3, MAP_RADIO_LOW, MAP_RADIO_HIGH);

// 0 or 1 for channels 5 or 6
ch5 = map(ch5, MIN_5, MAX_5, MAP_RADIO_LOW, MAP_RADIO_HIGH)/ 75;
ch6 = map(ch6, MIN_6, MAX_6, MAP_RADIO_LOW, MAP_RADIO_HIGH) / 75;


}

void loop()
{
 
checkTransmitter();//check the data being received by the transmitter.
Serial.println ("");
Serial.println ("data: ");

Serial.print ("ch1 ");
Serial.println (ch1);

Serial.print ("ch2 ");
Serial.println (ch2);

Serial.print ("ch3 ");
Serial.println (ch3);

Serial.print ("ch4 ");
Serial.println (ch4);




Serial.print ("ch5 ");
Serial.println (ch5);

Serial.print ("ch6 ");
Serial.println (ch6);


delay (500);
}
