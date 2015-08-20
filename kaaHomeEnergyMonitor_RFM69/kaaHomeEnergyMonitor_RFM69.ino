//------------------------------------------------------------------------------------------------------------------------------------------------
// emonGLCD Home Energy Monitor example
// to be used with nanode Home Energy Monitor example

// Uses power1 variable - change as required if your using different ports

// emonGLCD documentation http://openEnergyMonitor.org/emon/emonglcd

// RTC to reset Kwh counters at midnight is implemented is software. 
// Correct time is updated via NanodeRF which gets time from internet
// Temperature recorded on the emonglcd is also sent to the NanodeRF for online graphing

// GLCD library by Jean-Claude Wippler: JeeLabs.org
// 2010-05-28 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
//
// Authors: Glyn Hudson and Trystan Lea
// Part of the: openenergymonitor.org project
// Licenced under GNU GPL V3
// http://openenergymonitor.org/emon/license

// THIS SKETCH REQUIRES:

// Libraries in the standard arduino libraries folder:
//
//	- OneWire library	http://www.pjrc.com/teensy/td_libs_OneWire.html
//	- DallasTemperature	http://download.milesburton.com/Arduino/MaximTemperature
//                           or http://github.com/milesburton/Arduino-Temperature-Control-Library
//	- JeeLib		https://github.com/jcw/jeelib
//	- RTClib		https://github.com/jcw/rtclib
//	- GLCD_ST7565		https://github.com/jcw/glcdlib
//
// Other files in project directory (should appear in the arduino tabs above)
//	- icons.ino
//	- templates.ino
//
//-------------------------------------------------------------------------------------------------------------------------------------------------

#define MAX_TIME_TO_RECEIVE 600 // number of seconds to wait for first reception of time and usage data after reset takes place

#include <avr/wdt.h>
#include <EEPROMex.h>      //get it here: http://playground.arduino.cc/Code/EEPROMex
#include <RFM69.h>     //get it here: http://github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
#include <GLCD_ST7565.h>
#include <avr/pgmspace.h>
GLCD_ST7565 glcd;

#include <OneWire.h>		    // http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h>      // http://download.milesburton.com/Arduino/MaximTemperature/ (3.7.2 Beta needed for Arduino 1.0)

#include <RTClib.h>                 // Real time clock (RTC) - used for software RTC to reset kWh counters at midnight
#include <Wire.h>                   // Part of Arduino libraries - needed for RTClib
RTC_Millis RTC;

#define ONE_WIRE_BUS 5              // temperature sensor connection - hard wired 

#define NODEID        3    //unique for each node on same network
#define GATEWAYID     1  //assumed 1 in general
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "df39ea10cc412@1k" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    0 //set to 1 only for RFM69HW! Leave 0 if you have RFM69(C)W!
bool promiscuousMode = true; //set to 'true' to sniff all packets on the same network
bool collectMode = false; //set to 'true' only collect data and not respond to ACK_REQ
bool ackerror = false;
long randNumber;

struct configuration {
  byte frequency;
  byte isHW;
  byte nodeID;
  byte networkID;
  byte collectMode;                  //separators needed to keep strings from overlapping 
  byte promiscuousMode;              //separators needed to keep strings from overlapping
  char encryptionKey[16];
  byte separator1;
  char description[10];
  byte separator2;
} CONFIG;

#define ACK_TIME      30 // max # of ms to wait for an ack
#define LED_PIN     9       // activity LED, comment out to disable

//#define SERIAL_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    57600
#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); Serial.flush();delay(1);}
  #define DEBUGln(input) {Serial.println(input); Serial.flush();delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#define LED           9 // Moteinos have LEDs on D9
#define VERSION "\n[kaaNet_HEM_RFM69.2]\n"

unsigned long fast_update, slow_update;
long int last_time_update;
long int last_usage_update;
long int update_tobase_timer;
//long int debug_timer;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
double temp,maxtemp,mintemp;

RFM69 radio;


//---------------------------------------------------
// Data structures for transfering data between units
//---------------------------------------------------
typedef struct { byte packettype; int power1; } PayloadTX;         // neat way of packaging data for RF comms
PayloadTX emontx;

typedef struct { int temperature, lightlevel; } PayloadGLCD;
PayloadGLCD emonglcd;

int hour = 24, minute = 0;
double usekwh = 0;

const int greenLED=6;               // Green tri-color LED
const int redLED=9;                 // Red tri-color LED
const int LDRpin=4;    		    // analog pin of onboard lightsensor 
int cval_use;

//-------------------------------------------------------------------------------------------- 
// Flow control
//-------------------------------------------------------------------------------------------- 
unsigned long last_emontx;                   // Used to count time from last emontx update
unsigned long last_emonbase;                   // Used to count time from last emontx update

static void displayVersion () {
    DEBUGln(VERSION);
}

static void configDump() {
  displayVersion();
  EEPROM.readBlock(0, CONFIG);
  DEBUGln("Current coniguration:");
  DEBUG("NODEID:");DEBUGln(CONFIG.nodeID);
  DEBUG("NETWORKID:");DEBUGln(CONFIG.networkID);
  DEBUG("Description:");DEBUGln(CONFIG.description);
  DEBUG("Frequency:");DEBUG(CONFIG.frequency==RF69_433MHZ?"433":CONFIG.frequency==RF69_868MHZ?"868":"915");DEBUGln(" mhz");
  DEBUG("EncryptKey:");DEBUGln(CONFIG.encryptionKey);
  DEBUG("isHW:");DEBUGln(CONFIG.isHW);
  DEBUG("collectMode:");DEBUGln(CONFIG.collectMode);
  DEBUG("promiscuousMode:");DEBUGln(CONFIG.promiscuousMode);
}

void setup()
{
  #ifdef SERIAL_EN
    Serial.begin(SERIAL_BAUD);
  #endif
  EEPROM.readBlock(0, CONFIG);
  if (CONFIG.frequency!=RF69_433MHZ && CONFIG.frequency!=RF69_868MHZ && CONFIG.frequency!=RF69_915MHZ) // virgin CONFIG, expected [4,8,9]
  {
    DEBUGln("No valid config found in EEPROM, writing defaults");
      CONFIG.separator2=CONFIG.separator1=0;
      CONFIG.collectMode=0;
      CONFIG.promiscuousMode=0;
      CONFIG.frequency=FREQUENCY;
      CONFIG.description[0]=0;
      strcpy(CONFIG.encryptionKey, ENCRYPTKEY);
      CONFIG.isHW=IS_RFM69HW;
      CONFIG.nodeID=NODEID;
      CONFIG.networkID=NETWORKID;
      EEPROM.writeBlock(0, CONFIG);
  }
  
  radio.initialize(CONFIG.frequency,CONFIG.nodeID,CONFIG.networkID);
  radio.sleep();
  //radio.setPowerLevel(10);
  if (CONFIG.isHW) radio.setHighPower(); //use with RFM69HW ONLY!
  if (CONFIG.encryptionKey[0]!=0) radio.encrypt(CONFIG.encryptionKey);
  
  radio.writeReg( 0x03, 0x06); //19.2kBps
  radio.writeReg( 0x04, 0x83);
  radio.writeReg( 0x05, 0x01); //default:5khz, (FDEV + BitRate/2 <= 500Khz)
  radio.writeReg( 0x06, 0x9A);
  radio.writeReg( 0x19, 0x40 | 0x10 | 0x03); 
  radio.writeReg( 0x3d, 0xC0 | 0x02 | 0x00); //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
  configDump();
  promiscuousMode = (CONFIG.promiscuousMode == 1);
  radio.promiscuous(promiscuousMode);
  collectMode = (CONFIG.collectMode == 1);

  delay(100);				   //wait for RF to settle befor turning on display
  glcd.begin(0x18);
  glcd.backLight(200);
  
  sensors.begin();                         // start up the DS18B20 temp sensor onboard  
  sensors.requestTemperatures();
  temp = (sensors.getTempCByIndex(0));     // get inital temperture reading
  mintemp = temp; maxtemp = temp;          // reset min and max

  pinMode(greenLED, OUTPUT); 
  pinMode(redLED, OUTPUT); 
  DEBUGln("[kaaHomeEnergyMonitor_RFM69.1]");
  
  wdt_reset ();
 // wdt_enable (WDTO_8S);
// debug_timer = millis();
 last_time_update = millis();
 last_usage_update = millis();
 randomSeed(analogRead(0));
 randNumber = random(10);
}

//--------------------------------------------------------------------------------------------
// Loop
//--------------------------------------------------------------------------------------------
void loop()
{
  if (((millis() - last_time_update) / 1000 >= (MAX_TIME_TO_RECEIVE)) || ((millis() - last_usage_update) / 1000 >= (MAX_TIME_TO_RECEIVE)))
  {
    wdt_reset ();
    wdt_enable (WDTO_1S);
    while(1);
    //Serial.println('REBOOT...');
  }
//  if (((millis() - last_time_update) / 1000) != debug_timer / 1000) 
//  { debug_timer = (millis() - last_time_update);
//    Serial.println(debug_timer / 1000);
//    Serial.println(MAX_TIME_TO_RECEIVE);
//  }
  if (radio.receiveDone())
  {
      int node_id = (radio.TARGETID);
      DEBUG("rcv: "); DEBUG(radio.TARGETID);DEBUG("> ");DEBUG(radio.DATA[0]);DEBUG(",");DEBUG(radio.DATA[1]);DEBUG(",");DEBUG(radio.DATA[2]);DEBUG(": ");DEBUGln(radio.PAYLOADLEN);
      if ((node_id == RF69_BROADCAST_ADDR) && (radio.DATA[0] == 22) && radio.PAYLOADLEN == 7) //broadcast packet type 22 [time]
      {
        RTC.adjust(DateTime(2014, 5, 16, radio.DATA[1], radio.DATA[2], radio.DATA[3]));
        last_emonbase = millis();
        last_time_update = millis();
      }
      if ((node_id == RF69_BROADCAST_ADDR) && (radio.DATA[0] == 21) && radio.PAYLOADLEN == 6) //broadcast packet type 21 [verbruik]
      { DEBUGln("display"); emontx = *(PayloadTX*) radio.DATA; 
        last_emontx = millis();
        last_usage_update = millis();
      }
      if (radio.ACKRequested() && !collectMode)
      {
        radio.sendACK();
      } 
    }
  //--------------------------------------------------------------------------------------------
  // Display update every 200ms
  //--------------------------------------------------------------------------------------------
  if ((millis()-fast_update)>200)
  {
    fast_update = millis();
    
    DateTime now = RTC.now();
    int last_hour = hour;
    hour = now.hour();
    minute = now.minute();

    usekwh += (emontx.power1 * 0.2) / 3600000;
    if (last_hour == 23 && hour == 00) usekwh = 0;                //reset Kwh/d counter at midnight
    cval_use = cval_use + (emontx.power1 - cval_use)*0.50;        //smooth transitions
    if (millis()<(last_usage_update+1000)){
      draw_power_page_update( "ACTUEEL" ,cval_use, "", usekwh);
    }
    else
    {
      draw_power_page( "ACTUEEL" ,cval_use, "", usekwh);
    }
    if (millis()<(last_time_update+1000)){
      if (!ackerror) {
        draw_temperature_time_footer_update(temp, mintemp, maxtemp, hour,minute);
      }
      else {
        draw_temperature_time_footer_error(temp, mintemp, maxtemp, hour,minute); 
      }
    }
    else
    {
      if (!ackerror) {
        draw_temperature_time_footer(temp, mintemp, maxtemp, hour,minute);
      }
      else {
        draw_temperature_time_footer_error(temp, mintemp, maxtemp, hour,minute); 
      }
    }
    glcd.refresh();

    int LDR = analogRead(LDRpin);                     // Read the LDR Value so we can work out the light level in the room.
    int LDRbacklight = map(LDR, 0, 1023, 50, 250);    // Map the data from the LDR from 0-1023 (Max seen 1000) to var GLCDbrightness min/max
    LDRbacklight = constrain(LDRbacklight, 0, 255);   // Constrain the value to make sure its a PWM value 0-255
    if ((hour > 22) ||  (hour < 5)) glcd.backLight(150); else 
      glcd.backLight(LDRbacklight);  
  } 
  
  if ((millis()-slow_update)>(100000) + (randNumber * 1000))
  {
    slow_update = millis();

    sensors.requestTemperatures();
    temp = (sensors.getTempCByIndex(0));
    if (temp > maxtemp) maxtemp = temp;
    if (temp < mintemp) mintemp = temp;
   
    emonglcd.temperature = (int) (temp * 100);                          // set emonglcd payload
    int LDR = analogRead(LDRpin);                     // Read the LDR Value so we can work out the light level in the room.
    int LDRbacklight = map(LDR, 0, 1023, 50, 250);    // Map the data from the LDR from 0-1023 (Max seen 1000) to var GLCDbrightness min/max
    emonglcd.lightlevel = LDRbacklight;

    if (!(radio.sendWithRetry(GATEWAYID, &emonglcd, sizeof emonglcd))){
      ackerror = true;
      draw_temperature_time_footer_error(temp, mintemp, maxtemp, hour,minute); 
    }
    else {
      ackerror = false;
    }    
  }
}
