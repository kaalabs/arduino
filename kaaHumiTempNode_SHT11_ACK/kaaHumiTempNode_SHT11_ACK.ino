/*
  emonTH V1.4 Low Power DHT22 Humidity & Temperature & DS18B20 Temperature Node Example 

  Checkes at startup for presence of a DS18B20 temp sensor , DHT22 (temp + humidity) or both
  If it finds both sensors the temperature value will be taken from the DS18B20 (external) and DHT22 (internal) and humidity from DHT22
  If it finds only DS18B20 then no humidity value will be reported
  If it finds only a DHT22 then both temperature and humidity values will be obtained from this sesor
  
  Technical hardware documentation wiki: http://wiki.openenergymonitor.org/index.php?title=EmonTH
 
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson
  Builds upon JCW JeeLabs RF12 library, Arduino and Martin Harizanov's work

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
	- RFu JeeLib		https://github.com/openenergymonitor/RFu_jeelib   //library to work with CISECO RFu328 module
	- DHT22 Sensor Library  https://github.com/adafruit/DHT-sensor-library - be sure to rename the sketch folder to remove the '-'
        - OneWire library	http://www.pjrc.com/teensy/td_libs_OneWire.html
	- DallasTemperature	http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip

  Recommended node ID allocation
  ------------------------------------------------------------------------------------------------------------
  -ID-	-Node Type- 
  0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
  1-4     - Control nodes 
  5-10	- Energy monitoring nodes
  11-14	--Un-assigned --
  15-16	- Base Station & logging nodes
  17-30	- Environmental sensing nodes (temperature humidity etc.)
  31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
  -------------------------------------------------------------------------------------------------------------
*/
#define RF69_COMPAT 0

boolean debug=1;                                       //Set to 1 to few debug serial output, turning debug off increases battery life

#define RF_freq RF12_868MHZ                 // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 5;                               // EmonTH temperature RFM12B node ID - should be unique on network
const int networkGroup = 5;                // EmonTH RFM12B wireless network group - needs to be same as emonBase and emonGLCD
                                                                      // DS18B20 resolution 9,10,11 or 12bit corresponding to (0.5, 0.25, 0.125, 0.0625 degrees C LSB), lower resolution means lower power

const int time_between_readings= 1;                                   // in minutes
const int TEMPERATURE_PRECISION=12;                                   // 9 (93.8ms),10 (187.5ms) ,11 (375ms) or 12 (750ms) bits equal to resplution of 0.5C, 0.25C, 0.125C and 0.0625C
#define ASYNC_DELAY 750                                               // 9bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms

// See block comment above for library info
#include <avr/power.h>
#include <avr/sleep.h>
#include <RFu_JeeLib.h>                                                 
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PortsSHT11.h>
ISR(WDT_vect) { Sleepy::watchdogEvent(); }                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

// Hardwired emonTH pin allocations 
const int DS18B20_PWR=5;
const int LED=9;
const int BATT_ADC=1;

#define RETRY_PERIOD    10  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     5   // maximum number of times to retry
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define ONE_WIRE_BUS 19
#define SHT11_PORT 1  
#define RADIO_SYNC_MODE 3

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
boolean DS18B20;                                                      // create flag variable to store presence of DS18B20 

typedef struct {                                                      // RFM12B RF payload datastructure
  	  int temp;
          int temp_external;
          int humidity;    
          int battery;          	                                      
} Payload;
Payload emonth;

#if SHT11_PORT
    SHT11 sht11 (SHT11_PORT);
#endif

int numSensors; 
//addresses of sensors, MAX 4!!  
byte allAddress [4][8];                                              // 8 bytes per address

// spend a little time in power down mode while the SHT11 does a measurement
static void shtDelay () {
    dodelay(32); // must wait at least 20 ms
}

static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | nodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
    return 0;
}
//################################################################################################################################
//################################################################################################################################
void setup() {
//################################################################################################################################
  
  pinMode(LED,OUTPUT); digitalWrite(LED,HIGH);                       // Status LED on
   
  rf12_initialize(nodeID, RF_freq, networkGroup);                       // Initialize RFM12B
  
  // Send RFM12B test sequence (for factory testing)
  for (int i=10; i==0; i--)                                           
  {
    emonth.temp=i; 
    rf12_sendNow(0, &emonth, sizeof emonth);
    delay(100);
  }
  rf12_sendWait(2);
  emonth.temp=0;
  // end of factory test sequence
  
  rf12_sleep(RF12_SLEEP);
  
  //if (Serial) debug = 1; else debug=0;                              //if serial UART to USB is connected show debug O/P. If not then disable serial - DOES NOT WORK http://openenergymonitor.org/emon/node/3930
  
  if (debug==1)
  {
    Serial.begin(57600);
    Serial.println("emonTH"); 
    Serial.println("OpenEnergyMonitor.org");
    Serial.print("Node: "); 
    Serial.print(nodeID); 
    Serial.print(" Freq: "); 
    if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
    if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
    if (RF_freq == RF12_915MHZ) Serial.print("915Mhz"); 
    Serial.print(" Network: "); 
    Serial.println(networkGroup);
    delay(100);
  }
  
  pinMode(DS18B20_PWR,OUTPUT);
  pinMode(BATT_ADC, INPUT);

  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  ACSR |= (1 << ACD);                     // disable Analog comparator    
  if (debug==0) power_usart0_disable();   //disable serial UART
  power_twi_disable();                    //Disable the Two Wire Interface module.
  // power_timer0_disable();              //don't disable necessary for the DS18B20 library
  power_timer1_disable();
  power_spi_disable();
 
 
  //################################################################################################################################
  // Setup and for presence of DS18B20
  //################################################################################################################################
  digitalWrite(DS18B20_PWR, HIGH); delay(50); 
  sensors.begin();
  sensors.setWaitForConversion(false);                             //disable automatic temperature conversion to reduce time spent awake, conversion will be implemented manually in sleeping http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/ 
  numSensors=(sensors.getDeviceCount()); 
  
  byte j=0;                                        // search for one wire devices and
                                                   // copy to device address arrays.
  while ((j < numSensors) && (oneWire.search(allAddress[j])))  j++;
  digitalWrite(DS18B20_PWR, LOW);
  
  if (numSensors==0)
  {
    if (debug==1) Serial.println("No DS18B20 detected");
    DS18B20=0; 
  } 
  else 
  {
    DS18B20=1; 
    if (debug==1) {
      Serial.print("Detected "); Serial.print(numSensors); Serial.println(" DS18B20");
    }
    
  }
  if (debug==1) delay(200);
  
  //################################################################################################################################
   
  digitalWrite(LED,LOW);
} // end of setup


//################################################################################################################################
//################################################################################################################################
void loop()
//################################################################################################################################
{ 
    #if SHT11_PORT
      if (debug == 1){
          Serial.print("\n   SHT11.humi ");
          Serial.flush();
      }
      sht11.measure(SHT11::HUMI, shtDelay);        
      if (debug == 1){
        Serial.print("\n   SHT11.temp ");
        Serial.flush();
        delay(2);
      }
      sht11.measure(SHT11::TEMP, shtDelay);
      float h, t;
      sht11.calculate(h, t);
      int humi = h + 0.5, temp = 10 * t + 0.5;
    #endif
    if (debug == 1){
      Serial.print("\n   SHT11 temp read: ");
      Serial.println((float) t);
      Serial.flush();
      delay(2);
    }
    if (debug == 1){
      Serial.print("\n   SHT11 humi read: ");
      Serial.println((float) h);
      Serial.flush();
      delay(2);
    }
//  if (DS18B20==0)        //if neither DS18B20 or DHT22 is detected flash the LED then goto forever sleep
//  {
//    for (int i=0; i<20; i++)
//    {
//      digitalWrite(LED, HIGH); delay(200); digitalWrite(LED,LOW); delay(200);
//    }
//    cli();                                      //stop responding to interrupts 
//    Sleepy::powerDown();                        //sleep forever
//  }

  if (DS18B20==1)
  {
    digitalWrite(DS18B20_PWR, HIGH); dodelay(50); 
    for(int j=0;j<numSensors;j++) sensors.setResolution(allAddress[j], TEMPERATURE_PRECISION);      // and set the a to d conversion resolution of each.
    sensors.requestTemperatures();                                        // Send the command to get temperatures
    dodelay(ASYNC_DELAY); //Must wait for conversion, since we use ASYNC mode
    float temp=(sensors.getTempC(allAddress[0]));
    digitalWrite(DS18B20_PWR, LOW);
    emonth.temp=(temp*10);            // if DHT22 is not present assume DS18B20 is primary sensor (internal)
  }
    
  
  emonth.battery=int(analogRead(BATT_ADC)*0.03225806);                    //read battery voltage, convert ADC to volts x10
                                               
  
  if (debug==1) 
  {
    if (DS18B20)
    {
      Serial.print("DS18B20 Temperature: ");
      Serial.print(emonth.temp/10.0);
      Serial.print("C, ");
      Serial.flush();
      delay(2);
    }
        
    Serial.print("Battery voltage: ");  
    Serial.print(emonth.battery/10.0);
    Serial.println("V");
    Serial.flush();
    delay(100);
  }

  
  power_spi_enable(); 
  if (debug == 1){
    Serial.println("kaaNet transmission...");
    Serial.flush();
    delay(2);
  }
  for (byte i = 0; i < RETRY_LIMIT; ++i) {
        rf12_sleep(RF12_WAKEUP);
        rf12_sendNow(RF12_HDR_ACK, &emonth, sizeof emonth);
        rf12_sendWait(RADIO_SYNC_MODE);
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);

        if (acked) {
            if (debug==1){
                Serial.print(" ack ");
                Serial.println((int) i);
                Serial.flush();
            }
            return;
        }
        
        delay(RETRY_PERIOD * 100);
  }
  if (debug == 1){
    Serial.println("Going to sleep...");
    Serial.flush();
    delay(2);
  }
  power_spi_disable();  
  // digitalWrite(LED,HIGH);
  // dodelay(100);
  // digitalWrite(LED,LOW);  
  
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;   
  Sleepy::loseSomeTime(time_between_readings*60*1000);  
  //Sleepy::loseSomeTime(2000);
  ADCSRA=oldADCSRA; // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;
      
  Sleepy::loseSomeTime(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
      
  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

