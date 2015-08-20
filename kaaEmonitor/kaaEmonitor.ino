/// Small demo for the Relay Plug, receives wireless packets and sets relays.
// 2010-07-05 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// 2013-10-01 <remco@remcokortekaas.nl> fork relay_demo
// 2014-01-29 adjusted code to handle 3 ledbars / 4x8 bit shift registers + 2x1 status leds
// 2014-02-22 implemented differentation between status leds. red = Broadcast packet received, yellow = kaaEMonitor target data received

#include <JeeLib.h>

#define RADIO_SYNC_MODE 2

int clockPin = 16; //IC Pin 11, Yellow Jumper
int dataPin = 7; //IC Pin 14, Blue Jumper
int latchPin = 6; //IC Pin 12, Green Jumper

int DataLedState = LOW;
int BroadcastLedState = LOW;

long BlinkInterval = 500;
long prevDataLedBlinkMillis = 0;
long prevBroadcastLedBlinkMillis = 0;

struct {
  byte shift4;
  byte shift3;
  byte shift2;
  byte shift1;
  byte notinuse;
} lcd_data;

void UpdateLedDisplay(){
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, lcd_data.shift4);
      shiftOut(dataPin, clockPin, MSBFIRST, lcd_data.shift3);  
      shiftOut(dataPin, clockPin, MSBFIRST, lcd_data.shift2);
      shiftOut(dataPin, clockPin, MSBFIRST, lcd_data.shift1);
      digitalWrite(latchPin, HIGH);  
}

void SetDataLedOn(){
  lcd_data.shift4 = lcd_data.shift4 | 0x80;
  DataLedState = HIGH;
  prevDataLedBlinkMillis = millis();
}

void SetDataLedOff(){
  lcd_data.shift4 = lcd_data.shift4 & 0x7F;
  DataLedState = LOW;
}

void SetBroadcastLedOn(){
  lcd_data.shift4 = lcd_data.shift4 | 0x40;
  BroadcastLedState = HIGH;
  prevBroadcastLedBlinkMillis = millis();
}

void SetBroadcastLedOff(){
  lcd_data.shift4 = lcd_data.shift4 & 0xBF;
  BroadcastLedState = LOW;
}

void setup () {
    rf12_initialize(12, RF12_868MHZ, 5);
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, 0xFF);
    shiftOut(dataPin, clockPin, MSBFIRST, 0xFF);
    shiftOut(dataPin, clockPin, MSBFIRST, 0xFF);
    shiftOut(dataPin, clockPin, MSBFIRST, 0xFF);
    digitalWrite(latchPin, HIGH);
    delay(2000);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, 0x00);
    shiftOut(dataPin, clockPin, MSBFIRST, 0x00);
    shiftOut(dataPin, clockPin, MSBFIRST, 0x00);
    shiftOut(dataPin, clockPin, MSBFIRST, 0x00);
    digitalWrite(latchPin, HIGH);    
    lcd_data.shift1 = 0;
    lcd_data.shift2 = 0;
    lcd_data.shift3 = 0;
    lcd_data.shift4 = 0;
}

void loop () {
  unsigned long currentmillis = millis();
  if (currentmillis < prevDataLedBlinkMillis) prevDataLedBlinkMillis = 0;
  if (currentmillis < prevBroadcastLedBlinkMillis) prevBroadcastLedBlinkMillis = 0;
  if (DataLedState == HIGH){
    if (currentmillis - prevDataLedBlinkMillis >= BlinkInterval){ 
      SetDataLedOff();
      UpdateLedDisplay();
    }
  }
  if (BroadcastLedState == HIGH){
    if (currentmillis - prevBroadcastLedBlinkMillis >= BlinkInterval){ 
      SetBroadcastLedOff();
      UpdateLedDisplay();
    }
  }
  if (rf12_recvDone())
  {
    if (rf12_crc == 0) if ((rf12_hdr & RF12_HDR_DST) == RF12_HDR_DST) // no rf errors, ignore broadcasts
    {
        lcd_data.shift1 = rf12_data[0];
        lcd_data.shift2 = rf12_data[1];
        lcd_data.shift3 = rf12_data[2];
        lcd_data.shift4 = rf12_data[3];
        SetDataLedOn();
        if (BroadcastLedState == HIGH) lcd_data.shift4 = lcd_data.shift4 | 0x40;
        UpdateLedDisplay();
    } else
    if ((rf12_crc == 0) && ((rf12_hdr & RF12_HDR_DST) != RF12_HDR_DST)){ 
      SetBroadcastLedOn();
      UpdateLedDisplay();
    }
  }
}
