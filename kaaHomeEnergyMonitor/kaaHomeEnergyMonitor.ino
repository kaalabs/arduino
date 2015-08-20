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
#define RF69_COMPAT 0
#define MAX_TIME_TO_RECEIVE 600 // number of seconds to wait for first reception of time and usage data after reset takes place

#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <JeeLib.h>
#include <GLCD_ST7565.h>
#include <avr/pgmspace.h>
GLCD_ST7565 glcd;

#include <OneWire.h>		    // http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h>      // http://download.milesburton.com/Arduino/MaximTemperature/ (3.7.2 Beta needed for Arduino 1.0)

#include <RTClib.h>                 // Real time clock (RTC) - used for software RTC to reset kWh counters at midnight
#include <Wire.h>                   // Part of Arduino libraries - needed for RTClib
RTC_Millis RTC;

// RF12 configuration area
typedef struct {
    byte nodeId;            // used by rf12_config, offset 0
    byte group;             // used by rf12_config, offset 1
    byte format;            // used by rf12_config, offset 2
    byte output :2;         // 0 = dec, 1 = hex, 2 = dec+ascii, 3 = hex+ascii
    byte collect_mode :1;   // 0 = ack, 1 = don't send acks
    byte quiet_mode   :1;   // 0 = show all, 1 = show only valid packets
    byte spare_flags  :4;
    word frequency_offset;  // used by rf12_config, offset 4
    byte pad[RF12_EEPROM_SIZE-8];
    word crc;
} RF12Config;

static RF12Config config;

#define ONE_WIRE_BUS 5              // temperature sensor connection - hard wired 
#define DEBUG 0

unsigned long fast_update, slow_update;
long int last_time_update;
long int last_usage_update;
//long int debug_timer;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
double temp,maxtemp,mintemp;


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

static void loadConfig () {
    // eeprom_read_block(&config, RF12_EEPROM_ADDR, sizeof config);
    // this uses 166 bytes less flash than eeprom_read_block(), no idea why
    for (byte i = 0; i < sizeof config; ++i)
        ((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + i);
}

//--------------------------------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------------------------------
void setup()
{
  delay(500); 				   //wait for power to settle before firing up the RF//
//  rf12_initialize(MYNODE, RF_freq,group);
if (rf12_configSilent()) {
        loadConfig();
  } else {
      while (1);
  }
  delay(100);				   //wait for RF to settle befor turning on display
  glcd.begin(0x18);
  glcd.backLight(200);
  
  sensors.begin();                         // start up the DS18B20 temp sensor onboard  
  sensors.requestTemperatures();
  temp = (sensors.getTempCByIndex(0));     // get inital temperture reading
  mintemp = temp; maxtemp = temp;          // reset min and max

  pinMode(greenLED, OUTPUT); 
  pinMode(redLED, OUTPUT);
 #if DEBUG 
  Serial.begin(57600);
    Serial.println("[kaaHomeEnergyMonitor.2]");
 #endif
  wdt_reset ();
 // wdt_enable (WDTO_8S);
// debug_timer = millis();
 last_time_update = millis();
 last_usage_update = millis();
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
  if (rf12_recvDone())
  {
    if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)  // clean receive
    {
      int node_id = (rf12_hdr & 0x1F);
      if ((rf12_hdr & RF12_HDR_DST) == 0 && (node_id == 31) && (rf12_data[0] == 22) && rf12_len == 4) //broadcast packet type 22 [time]
      {
        RTC.adjust(DateTime(2014, 5, 16, rf12_data[1], rf12_data[2], rf12_data[3]));
        last_emonbase = millis();
        last_time_update = millis();
      }
      if ((rf12_hdr & RF12_HDR_DST) == 0 && (node_id == 31) && (rf12_data[0] == 21) && rf12_len == 3) //broadcast packet type 21 [verbruik]{
      { emontx = *(PayloadTX*) rf12_data; 
        last_emontx = millis();
        last_usage_update = millis();
      } 
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
      draw_temperature_time_footer_update(temp, mintemp, maxtemp, hour,minute);
    }
    else
    {
      draw_temperature_time_footer(temp, mintemp, maxtemp, hour,minute);
    }
    glcd.refresh();

    int LDR = analogRead(LDRpin);                     // Read the LDR Value so we can work out the light level in the room.
    int LDRbacklight = map(LDR, 0, 1023, 50, 250);    // Map the data from the LDR from 0-1023 (Max seen 1000) to var GLCDbrightness min/max
    LDRbacklight = constrain(LDRbacklight, 0, 255);   // Constrain the value to make sure its a PWM value 0-255
    if ((hour > 22) ||  (hour < 5)) glcd.backLight(150); else 
      glcd.backLight(LDRbacklight);  
  } 
  
  if ((millis()-slow_update)>100000)
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
#if DEBUG
    Serial.println(LDRbacklight,DEC);
    Serial.flush();
#endif    
    rf12_sendNow(0, &emonglcd, sizeof emonglcd);                     //send temperature data via RFM12B using new rf12_sendNow wrapper -glynhudson
    rf12_sendWait(2);    
  }
}
