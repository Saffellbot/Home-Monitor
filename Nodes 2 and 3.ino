//Code was written by Brandon Saffell for use in a home automation system.
//Special thanks to Lowpower Labs for moteinos and RFM69 libraries!

//Used in Node 2 and 3////
//Node 2 = Kitchen//
//Node 3 = PC Room//

// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 library at: https://github.com/LowPowerLab/
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <DHT.h>  //For reading from DHT22
#include <LDR10k.h>  //For reading 10 LDR

///RFM 69 & Moteino Settings//
#define NODEID        2    //Node 2 is Kitchen, Node 3 is PC room.
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW   //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENCRYPTKEY    "16CHARACTERS1234" //exactly the same 16 characters/bytes on all nodes!
#define ACK_TIME      30 // max # of ms to wait for an ack
#define ENABLE_ATC

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif
byte sendSize=0;

#define SERIAL_BAUD   19200

//LDR Setup//
const int LDRPIN=A0;
const int LDRRESISTOR=9810;  //Series resistor for the LDR. //Node 2 - 9810, Node 3, 9710
float ldrFiltered=0.0;
float ldrFilterFactor=0.008;
unsigned long lastLDRUpdate=0;
unsigned long LDRSAMPLETIME=1000;

//LDR Open HAB Serial Output Parameters//
const char SENSORNAME1[3]="Li";
float sensorValue1=0.00;
const byte PRECISION1 = 2;  //Used for dtostrf() later.
char floatBuffer1[PRECISION1+4];

//DHT 22 Setup//
#define DHTPIN 4
#define DHTTYPE DHT22
float dhtTemperatureFiltered=0.0;
float dhtTemperatureFilterFactor=0.02;
float dhtHumidityFiltered=0.0;
float dhtHumidityFilterFactor=0.02;

//DHT Temperature Open HAB Serial Output Parameters//
const char SENSORNAME2[13]="Te";
float sensorValue2=0.00;
const byte PRECISION2 = 2;  //Used for dtostrf() later.
char floatBuffer2[PRECISION2+4];

//DHT Temperature Open HAB Serial Output Parameters//
const char SENSORNAME3[3]="Hu";
float sensorValue3=0.00;
const byte PRECISION3 = 2;  //Used for dtostrf() later.
char floatBuffer3[PRECISION3+4];

//Combined DHT Parameters//
const unsigned long DHTSAMPLETIME=29000;
unsigned long lastDHTUpdate=0;

//Radio Temperature//
const char SENSORNAME4[3]="RT";
int sensorValue4=0;


//Signal Strength//
const char SENSORNAME5[3]="SS";
int sensorValue5=0;

//Transmit Level//
const char SENSORNAME6[3]="TL";
int sensorValue6=0;

//DELETED / FUTURE USE//
/* const char SENSORNAME7[3]="??";
float sensorValue7=0.00;
const byte PRECISION7=2;
char floatBuffer7[PRECISION7+4];
*/

//Communications Control/////////////////////////////////////////////////////////////////////////////////
const unsigned long TRANSMITTIME=30000; //frequency that the communications are updated in ms.
unsigned long lastTransmit=0;
int transmitCounter=0;

//Open HAB Serial Transformation Setup//
char transmitBuffer[35];

//Make Objects//
LDR10k ldr(LDRPIN,LDRRESISTOR);
DHT dht(DHTPIN, DHTTYPE);
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif
//Support Functions//
void serialTransmit(); //Sends information off to Openhab
void getDHT(); //Reads from the DHT
void getLDR();  //Reads from the LDR
void Blink(byte, int); //Blinks the onbaord LED

void setup()
{
//RFM69 Setup//
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  #ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
  #endif
  radio.encrypt(ENCRYPTKEY);
  sprintf(transmitBuffer, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(transmitBuffer);
  
//ATC Setup//
  #ifdef ENABLE_ATC
  radio.enableAutoPower(-70);
  #endif
  #ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
  #endif
//DHT Setup//
  dht.begin();
//LDR Filter Base//
  ldrFiltered=ldr.readLDR();
  
  delay(2000);  //Allows time for the DHT to stabilize before reading it.

  dhtTemperatureFiltered=dht.readTemperature(true);
  
  delay(2000);  //Allows time for the DHT to stabilize before reading it.
	
  dhtHumidityFiltered=dht.readHumidity();
  
}

void loop()
{
//Handle Incoming Radio Information//
  if (radio.receiveDone())  
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

  if (radio.ACKRequested())    
  	{
      radio.sendACK();
      Serial.print(" - ACK sent");
  	}
    Blink(LED,3);
    Serial.println();
  }
	
  if(millis()-lastLDRUpdate>=LDRSAMPLETIME)
  {
    getLDR();
  }

  if(millis()-lastDHTUpdate>=DHTSAMPLETIME)
  {
  	getDHT();
  }
 
  if(millis()-lastTransmit>=TRANSMITTIME)
  {
 	serialTransmit();
  }
	
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

void getLDR(){ 
	float tempLDR=ldr.readLDR();
  	ldrFiltered=(ldrFilterFactor*tempLDR)+((1-ldrFilterFactor)*ldrFiltered);
    lastLDRUpdate=millis();  //Reset Time
   return;
 }
 
 void getDHT(){  
	float tempDHTTemperature=dht.readTemperature(true);  //True returns a temperature in F. Without an argument passed it returns in Celsuis.
    dhtTemperatureFiltered=(dhtTemperatureFilterFactor*tempDHTTemperature)+((1-dhtTemperatureFilterFactor)*dhtTemperatureFiltered);
  	
  	float tempDHTHumidity=dht.readHumidity();
  	dhtHumidityFiltered=(dhtHumidityFilterFactor*tempDHTHumidity)+((1-dhtHumidityFilterFactor)*dhtHumidityFiltered);
    
    lastDHTUpdate=millis();
}

void serialTransmit(){
	
	//LDR
	if (transmitCounter==1||transmitCounter==5||transmitCounter==9||transmitCounter==13||transmitCounter==17) //Sends out LDR every 2 minutes.
	{
	sensorValue1=ldrFiltered;  //This step is really uncessary and redundant.
	dtostrf(sensorValue1, PRECISION1+3, PRECISION1, floatBuffer1); //Convert the float to a string.
  	sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME1,floatBuffer1);  //Format the string to what I use in OpenHAB.
  	Serial.println(transmitBuffer);  //Debug
  	sendSize = strlen(transmitBuffer);  //Send it out
  	if (radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize))
   		Serial.print("LDR ok!");
  	else Serial.print(" nothing...");
  		Serial.println();
	}
  	
  	//DHT Temperature
	if (transmitCounter==2||transmitCounter==6||transmitCounter==10||transmitCounter==14||transmitCounter==18)
	{
  	sensorValue2=dhtTemperatureFiltered;
  	dtostrf(sensorValue2, PRECISION2+3, PRECISION2, floatBuffer2);
  	sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME2,floatBuffer2);
  	Serial.println(transmitBuffer);
  	sendSize = strlen(transmitBuffer);
  	if (radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize))
   		Serial.print("Temperature ok!");
  	else Serial.print(" nothing...");
  		Serial.println();
	}
  	
  	//DHT Humidity
  	if (transmitCounter==3|transmitCounter==7||transmitCounter==11||transmitCounter==15||transmitCounter==19)
  	{
  	sensorValue3=dhtHumidityFiltered;
  	dtostrf(sensorValue3, PRECISION3+3, PRECISION3, floatBuffer3);
  	sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME3,floatBuffer3);
  	Serial.println(transmitBuffer);
  	sendSize = strlen(transmitBuffer);
  	if (radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize))
   		Serial.print("Humidity ok!");
  	else Serial.print(" nothing...");
  		Serial.println();
  	}
  	
  	//Radio Transmit Level
  	if (transmitCounter==4)  //Every 10 minutes, used for diagnosis.
  	{
  	sensorValue6=radio._transmitLevel;
  	sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME6,sensorValue6);
  	Serial.println(transmitBuffer);
  	sendSize = strlen(transmitBuffer);
  	if (radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize))
   		Serial.print(" ok!");
  	else Serial.print(" nothing...");
  		Serial.println();
  	}
  	
  	//Unused, shell is kept for future use.
  	/*
  	if (transmitCounter==8)
  	{
  	sensorValue7=????;
  	dtostrf(sensorValue7, PRECISION7+3, PRECISION7, floatBuffer7);
  	sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME7,floatBuffer7);
  	Serial.println(transmitBuffer);
  	sendSize = strlen(transmitBuffer);
  	if (radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize))
   		Serial.print(" ok!");
  	else Serial.print(" nothing...");
  		Serial.println();
  	}
  	*/
  	
  	//Enclosure Temperature
  	if (transmitCounter==12)  //Every 10 minutes, used for diagnosis.
  	{
  	sensorValue4=radio.readTemperature(0)*1.8+32;
  	sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME4,sensorValue4);
  	Serial.println(transmitBuffer);
  	sendSize = strlen(transmitBuffer);
  	if (radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize))
   		Serial.print(" ok!");
  	else Serial.print(" nothing...");
  		Serial.println();
  	sensorValue5=radio.RSSI; //Moved to this block to obtain the value as close to a transmission as possible. Still sent much later.
  	}	
  	
  	//Signal Strength
  	if (transmitCounter==20)  //Every 10 minutes, used for diagnosis.
  	{
  	sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME5,sensorValue5);
  	Serial.println(transmitBuffer);
  	sendSize = strlen(transmitBuffer);
  	if (radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize))
   		Serial.print(" ok!");
  	else Serial.print(" nothing...");
  		Serial.println();
  	transmitCounter=0;  //Last transmit in the loop.
  	}
  	
  	transmitCounter++;  //Reset the loop.
  	
  	Blink(LED,3);
  	
  	lastTransmit=millis(); //Rest timer.
	
}
