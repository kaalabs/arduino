
#define RF69_COMPAT 0
#include <JeeLib.h>
#include <RTClib.h>                 // Real time clock (RTC) - used for software RTC to reset kWh counters at midnight
#include <Wire.h>                   // Part of Arduino libraries - needed for RTClib
#include "LedControl.h"

RTC_Millis RTC;

LedControl leddisplay=LedControl(6,4,5,1); //pin 6 = DIN, pin 4 = CLK, pin 5 = LOADCS
long int last_time_update;
unsigned long displaydelaytime=250; //delaytime between display updates

//--------------------------------------------------------------------------------------------
// RFM12B Settings
//--------------------------------------------------------------------------------------------
#define MYNODE 21            // Should be unique on network, node ID 30 reserved for base station
#define RF_freq RF12_868MHZ     // frequency - match to same frequency as RFM12B module (change to 868Mhz or 915Mhz if appropriate)
#define group 5


//---------------------------------------------------
// Data structures for transfering data between units
//---------------------------------------------------

int hour = 24, minute = 0, second = 0;


void setup() {
  delay(500); 				   //wait for power to settle before firing up the RF
  Serial.begin(57600);
  Serial.println("[kaaLEDclock.1]");
  rf12_initialize(MYNODE, RF_freq,group);
  delay(100);				   //wait for RF to settle befor turning on display
  last_time_update = millis();
  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  leddisplay.shutdown(0,false);
  /* Set the brightness to a medium values */
  leddisplay.setIntensity(0,15);
  /* and clear the display */
  leddisplay.clearDisplay(0);
}

void leddisplay_refresh(int h,int m,int s, byte update) {
 
int ones;
int tens;

ones=h%10;
h=h/10;
tens=h%10;

//Now print the hours
leddisplay.setDigit(0,0,(byte)tens,false);
leddisplay.setDigit(0,1,(byte)ones,false);

ones=m%10;
m=m/10;
tens=m%10;

//Now print the minutes
leddisplay.setDigit(0,2,(byte)tens,false);
leddisplay.setDigit(0,3,(byte)ones,(update == 0)); //flashing decimal point means remote time sync update received

ones=s%10;
s=s/10;
tens=s%10;

//Now print the seconds
leddisplay.setDigit(0,4,(byte)tens,false);
leddisplay.setDigit(0,5,(byte)ones,false);

delay(displaydelaytime);
}

void loop() {
  if (rf12_recvDone())
  {
    if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)  // clean broadcasted receive
    {
      int node_id = (rf12_hdr & 0x1F);
      if ((rf12_hdr & RF12_HDR_DST) == 0 && (node_id == 31) && (rf12_data[0] == 22) && rf12_len == 4) //broadcast packet type 22 [time]
      {
        RTC.adjust(DateTime(2014, 5, 16, rf12_data[1], rf12_data[2], rf12_data[3]));
        last_time_update = millis();
      }
    }
  }
  
  DateTime now = RTC.now();

  hour = now.hour();
  minute = now.minute();
  second = now.second();
  
  if (millis()<(last_time_update+1000)){
      leddisplay_refresh(hour,minute,second,1);  //flash decimal point for 1 second to indicate remote time sync update
    }
    else
    {
      leddisplay_refresh(hour,minute,second,0);
    }
    
}
