/// @dir roomNode
/// New version of the Room Node (derived from rooms.pde).
// 2010-10-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// see http://jeelabs.org/2010/10/20/new-roomnode-code/
// and http://jeelabs.org/2010/10/21/reporting-motion/

// The complexity in the code below comes from the fact that newly detected PIR
// motion needs to be reported as soon as possible, but only once, while all the
// other sensor values are being collected and averaged in a more regular cycle.

// 2013-09-14 kaaRoomNode 1v1 - added DS18B20 Thermometer support
// 2013-09-15 Implemented interim measure (shortcut) to give humidity trigger a chance under constantly firing PIR triggers (because bathroom ventilator also needs to be fired while standing under the shower)
// 2013-09-23 timing tuned
// 2013-10-18 more PIR timings tuned
// 2013-10-18 added One Wire init debug info and set resolution
// 2013-10-20 changed DS18B20 resolution to 12 bits for stability reasons
// 2013-10-21 added error-correction smoothing on large DS18B20 temperature delta (> 10) readings
// 2013-11-18 tripled sensitivity of humidity trigger (ventilator ON)
// 2013-11-18 tripled flatliner treshold to keep vent running longer before declaring humi stable and turning off
// 2013-12-30 switched to last(payload).humi (instead of payload.humi) as treshold to switch off bathroom ventilator
// 2013-12-30 raised flatliner treshold to about 5 minutes (30*10 seconds)
// 2014-01-07 added DS18B20 mandatory read-delay
// 2014-01-25 decreased led blink counter
// 2014-02-21 corrected rf12 header with destination bit set to 1 for controlling the relaynode


#include <JeeLib.h>
#include <PortsSHT11.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <OneWire.h>
#include <DallasTemperature.h>


#define SERIAL  0   // set to 1 to also report readings on the serial port
#define DEBUG   0   // set to 1 to display each loop() run and PIR trigger

#define SHT11_PORT  1   // defined if SHT11 is connected to a port
//#define ONE_WIRE_BUS_PORT 4   // defined if DS18B20 is connected to Arduino PIN
#define LDR_PORT    4   // defined if LDR is connected to a port's AIO pin
#define PIR_PORT    4   // defined if PIR is connected to a port's DIO pin
#define RELAY_NODEID  3
#define LED_PORT 6   // defined if LED is connected to Arduino PIN

#if SHT11_PORT
   #define MEASURE_PERIOD  63 // how often to measure, in tenths of seconds for a humidity node (bathroom vent triggering vent)
#else
   #define MEASURE_PERIOD 593  // normal roomnode measuring period
#endif

#define RETRY_PERIOD    30  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     10   // maximum number of times to retry
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define REPORT_EVERY    5   // report every N measurement cycles
#define SMOOTH          3   // smoothing factor used for running averages
#define FLATLINER_COUNT 30   // how many measurements before declaring humidity stable

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

// The scheduler makes it easy to perform various tasks at various times:

enum { MEASURE, REPORT, TASK_END };

static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);

// Other variables used in various places in the code:

static byte reportCount;    // count up until next report, i.e. packet send
static byte myNodeID;       // node ID used for this unit
static byte flatlinerindex;
static int loopcounter;
static byte humion = 0;

// This defines the structure of the packets which get sent out by wireless:

struct {
    byte light;     // light sensor: 0..255
    byte moved :1;  // motion detector: 0..1
    byte humi  :7;  // humidity: 0..100
    int temp   :10; // temperature: -500..+500 (tenths)
    byte lobat :1;  // supply voltage dropped under 3.1V: 0..1
} payload;

struct {
     byte setrelay1;
     byte setrelay2;
     byte dest;
} relaytrigger;

struct {
    byte light;     // light sensor: 0..255
    byte moved :1;  // motion detector: 0..1
    byte humi  :7;  // humidity: 0..100
    int temp   :10; // temperature: -500..+500 (tenths)
    byte lobat :1;  // supply voltage dropped under 3.1V: 0..1
} lastpayload;


// Conditional code, depending on which sensors are connected and how:

#if SHT11_PORT
    SHT11 sht11 (SHT11_PORT);
#endif

#if LDR_PORT
    Port ldr (LDR_PORT);
#endif

#if PIR_PORT
    #define PIR_HOLD_TIME   30  // hold PIR value this many seconds after change
    #define PIR_PULLUP      0   // set to one to pull-up the PIR input pin
    #define PIR_INVERTED    0   // 0 or 1, to match PIR reporting high or low
    
    /// Interface to a Passive Infrared motion sensor.
    class PIR : public Port {
        volatile byte value, changed;
        volatile uint32_t lastOn;
    public:
        PIR (byte portnum)
            : Port (portnum), value (0), changed (0), lastOn (0) {}

        // this code is called from the pin-change interrupt handler
        void poll() {
            // see http://talk.jeelabs.net/topic/811#post-4734 for PIR_INVERTED
            byte pin = digiRead() ^ PIR_INVERTED;
            // if the pin just went on, then set the changed flag to report it
            if (pin) {
                if (!state())
                    changed = 1;
                lastOn = millis();
            }
            value = pin;
        }

        // state is true if curr value is still on or if it was on recently
        byte state() const {
            byte f = value;
            if (lastOn > 0)
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    if (millis() - lastOn < 1000 * PIR_HOLD_TIME)
                        f = 1;
                }
            return f;
        }

        // return true if there is new motion to report
        byte triggered() {
            byte f = changed;
            changed = 0;
            return f;
        }
    };

    PIR pir (PIR_PORT);

    // the PIR signal comes in via a pin-change interrupt
    ISR(PCINT2_vect) { pir.poll(); }
#endif

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

#if ONE_WIRE_BUS_PORT
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(ONE_WIRE_BUS_PORT);
  // Pass our oneWire reference to Dallas Temperature. 
  DallasTemperature sensors(&oneWire);
  DeviceAddress insideThermometer;
#endif

// utility code to perform simple smoothing as a running average
static int smoothedAverage(int prev, int next, byte firstTime =0) {
    if (firstTime) {
        #if DEBUG
          Serial.print("\n   Smoothing initialized ");
          serialFlush();
        #endif
      return next;
    }
    return ((SMOOTH - 1) * prev + next + SMOOTH / 2) / SMOOTH;
}

// spend a little time in power down mode while the SHT11 does a measurement
static void shtDelay () {
    Sleepy::loseSomeTime(32); // must wait at least 20 ms
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
    return 0;
}

// readout all the sensors and other values
static void doMeasure() {
    #if SHT11_PORT
      byte firstTime = payload.humi == 0; // special case to init running avg
    #else
      byte firstTime = payload.temp == 0;
    #endif
    payload.lobat = rf12_lowbat();
    
    #if SHT11_PORT
#ifndef __AVR_ATtiny84__
        #if DEBUG
          Serial.print("\n   SHT11.humi ");
          serialFlush();
        #endif
        sht11.measure(SHT11::HUMI, shtDelay);        
        #if DEBUG
          Serial.print("\n   SHT11.temp ");
          serialFlush();
        #endif
        sht11.measure(SHT11::TEMP, shtDelay);
        float h, t;
        sht11.calculate(h, t);
        int humi = h + 0.5, temp = 10 * t + 0.5;
#else
        //XXX TINY!
        int humi = 50, temp = 25;
#endif
        #if DEBUG
          Serial.print("\n   SHT11 temp read: ");
          Serial.print((float) t);
          serialFlush();
        #endif
        #if DEBUG
          Serial.print("\n   SHT11 humi read: ");
          Serial.print((float) h);
          serialFlush();
        #endif
        payload.humi = smoothedAverage(payload.humi, humi, firstTime);
        payload.temp = smoothedAverage(payload.temp, temp, firstTime);
        if ((lastpayload.humi == payload.humi) && !firstTime) {
          flatlinerindex = flatlinerindex + 1;
        }
        else {
          flatlinerindex = 0;
        }
        lastpayload.humi = payload.humi;
        lastpayload.temp = payload.temp;
    #endif
    
    #if LDR_PORT
        #if DEBUG
          Serial.print("\n   LDR ");
          serialFlush();
        #endif        
        ldr.digiWrite2(1);  // enable AIO pull-up
        byte light = ~ ldr.anaRead() >> 2;
        ldr.digiWrite2(0);  // disable pull-up to reduce current draw
        #if DEBUG
          Serial.print("\n   LDR raw output: ");
          Serial.print((byte) light);
        #endif
        payload.light = smoothedAverage(payload.light, light, firstTime);
    #endif

    #if PIR_PORT
        #if DEBUG
          Serial.print("\n   PIR.state ");
          serialFlush();
        #endif
        payload.moved = pir.state();
    #endif
    
    #if ONE_WIRE_BUS_PORT
        float t;
        lastpayload.humi = payload.humi;
        lastpayload.temp = payload.temp;
        #if DEBUG
          Serial.print("\n   DS18B20.request ");
          serialFlush();
        #endif        
        sensors.requestTemperatures();
		Sleepy::loseSomeTime(750); // 9bit requires 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms
        t = sensors.getTempCByIndex(0);
//        if (!firstTime) { t = 1;}              // only for DS18B20 error-correction smoothing DEBUG situations
        #if DEBUG
          Serial.print("\n   DS18B20 temp read: ");
          Serial.print((float) t);
          serialFlush();
        #endif
        int temp = 10 * t + 0.5;
        if (abs(temp-lastpayload.temp)> 10 && (!firstTime)) {
          Serial.print("\n   DS18B20 error-correction smoothing: ");
          Serial.print((float) t);
          Serial.print(" -> ");
          temp = 0.85 * lastpayload.temp + 0.15 * temp;
          Serial.print((int) temp);
          serialFlush();
        }
        payload.temp = smoothedAverage(payload.temp, temp, firstTime);
        lastpayload.temp = payload.temp;
        #if DEBUG
          Serial.print("\n   DS18B20.depower ");
          serialFlush();
        #endif        
        oneWire.depower();
    #endif
  
    
  #if RELAY_NODEID && SHT11_PORT  
    if (lastpayload.humi == 0) {
      relaytrigger.setrelay1 = 0;
      relaytrigger.setrelay2 = 0;          
      doTriggerRelay();
    }  
    if (lastpayload.humi != 0 && (((payload.humi - lastpayload.humi) >= 2) || payload.humi >= 85) && relaytrigger.setrelay1 == 0) {
      relaytrigger.setrelay1 = 1;
      relaytrigger.setrelay2 = 1;
      relaytrigger.dest = RELAY_NODEID;
      doTriggerRelay();
      humion = lastpayload.humi;
    }
    else {
      if ((flatlinerindex == FLATLINER_COUNT) && (payload.humi <= (humion+5)) && (relaytrigger.setrelay1 == 1)) {
        relaytrigger.setrelay1 = 0;
        relaytrigger.setrelay2 = 0;
        relaytrigger.dest = RELAY_NODEID;
        doTriggerRelay();
        flatlinerindex = 0;
      }
    }
  #endif    
  #if LED_PORT
    shortblink(LED_PORT);
  #endif
}


static void serialFlush () {
    #if ARDUINO >= 100
        Serial.flush();
    #endif  
    delay(2); // make sure tx buf is empty before going back to sleep
}

// periodic report, i.e. send out a packet and optionally report on serial port
static void doReport() {
    rf12_sleep(RF12_WAKEUP);
    rf12_sendNow(0, &payload, sizeof payload);
    rf12_sendWait(RADIO_SYNC_MODE);
    rf12_sleep(RF12_SLEEP);
    #if SERIAL
        Serial.println();
        Serial.print("\n   ROOM broadcast: ");
        Serial.print((int) payload.light);
        Serial.print("l ");
        Serial.print((int) payload.moved);
        Serial.print("m ");
        Serial.print((int) payload.humi);
        Serial.print("h ");
        Serial.print((int) payload.temp);
        Serial.print("t ");
        Serial.print((int) payload.lobat);
        Serial.print("b ");
        Serial.println();
        serialFlush();
    #endif    
    #if DEBUG && RELAY_NODEID
        Serial.print("\n   RELAY status: ");
        Serial.print((byte) relaytrigger.setrelay1);
        Serial.print("r1 ");
        Serial.print((byte) relaytrigger.setrelay2);
        Serial.print("r2 ");
        Serial.println();
        serialFlush();
    #endif
    #if LED_PORT
      blink(LED_PORT);
    #endif
}

#if RELAY_NODEID
  static void doTriggerRelay() {
      rf12_sleep(RF12_WAKEUP);
      rf12_sendNow(RELAY_NODEID & 0x40, &relaytrigger, sizeof relaytrigger);
      rf12_sendWait(RADIO_SYNC_MODE);
      rf12_sleep(RF12_SLEEP);
      #if SERIAL
        Serial.println();
        Serial.print("\n   RELAY broadcast: ");
        Serial.print((byte) relaytrigger.setrelay1);
        Serial.print("r1 ");
        Serial.print((byte) relaytrigger.setrelay2);
        Serial.print("r2 ");
        Serial.println();
        serialFlush();
     #endif
}
#endif

// send packet and wait for ack when there is a motion trigger
static void doTrigger() {
    #if DEBUG
       Serial.print("\n   PIR.broadcast: ");
    #endif
    for (byte i = 0; i < RETRY_LIMIT; ++i) {
        #if DEBUG
          Serial.print((byte) i);
          Serial.print(" ");
          serialFlush();
        #endif
        rf12_sleep(RF12_WAKEUP);
        rf12_sendNow(RF12_HDR_ACK, &payload, sizeof payload);
        rf12_sendWait(RADIO_SYNC_MODE);
        #if DEBUG
          Serial.print(" waitForAck ");
        #endif
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);
        if (acked) {
            #if DEBUG
                Serial.print(" ack\n");
                serialFlush();
            #endif
            #if SHT11_PORT
              doMeasure();
            #endif
            // reset scheduling to start a fresh measurement cycle
            scheduler.timer(MEASURE, MEASURE_PERIOD);
            return;
        }
        delay(RETRY_PERIOD * 100);
    }
    scheduler.timer(MEASURE, MEASURE_PERIOD);
    #if DEBUG
        Serial.println(" no ack!");
        serialFlush();
    #endif
}

void shortblink (byte pin) {
    for (byte i = 0; i < 2; ++i) {
        delay(80);
        digitalWrite(pin, !digitalRead(pin));
    }
}


void blink (byte pin) {
    for (byte i = 0; i < 2; ++i) {
        delay(100);
        digitalWrite(pin, !digitalRead(pin));
    }
}

#if ONE_WIRE_BUS_PORT
  void printAddress(DeviceAddress deviceAddress)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
    } 
   }
#endif

void setup () {
  flatlinerindex = 0;
  loopcounter = 0;
  #if LED_PORT
    pinMode(LED_PORT, OUTPUT);
    blink(LED_PORT);
  #endif
  #if SERIAL || DEBUG
        Serial.begin(57600);
        Serial.print("\n[kaaRoomNode.1");
        #if SHT11_PORT
          Serial.print("|HUMI.TEMP");
        #endif
        #if PIR_PORT
          Serial.print("|MOTION");
        #endif
        #if ONE_WIRE_BUS_PORT
          Serial.print("|TEMP");
        #endif
        #if LDR_PORT
          Serial.print("|LIGHT");
        #endif
        #if LED_PORT
          Serial.print("|LED");
        #endif
        #if RELAY_NODEID  
          Serial.print("|RELAY:");
          Serial.print((byte) RELAY_NODEID);
        #endif  
        Serial.print("]");
        myNodeID = rf12_config();
        serialFlush();
  #else
     myNodeID = rf12_config(0); // don't report info on the serial port
  #endif
    
  rf12_sleep(RF12_SLEEP); // power down
    
  #if PIR_PORT
       pir.digiWrite(PIR_PULLUP);
    #ifdef PCMSK2
      bitSet(PCMSK2, PIR_PORT + 3);
      bitSet(PCICR, PCIE2);
    #else
        //XXX TINY!
    #endif
  #endif

  #if ONE_WIRE_BUS_PORT
    #if DEBUG
      Serial.println("Dallas Temperature IC Control Library");
      // locate devices on the bus
      Serial.print("Locating devices...");
    #endif
    sensors.begin();
    #if DEBUG
      Serial.print("Found ");
      Serial.print(sensors.getDeviceCount(), DEC);
      Serial.println(" devices.");

      // report parasite power requirements
      Serial.print("Parasite power is: "); 
      if (sensors.isParasitePowerMode()) Serial.println("ON");
      else Serial.println("OFF");
      if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
      Serial.print("Device 0 Address: ");
      printAddress(insideThermometer);
      Serial.println();
    #endif
    // set the resolution to 12 bit (seems to be most stable as IC has more time to proces)
    sensors.setResolution(insideThermometer, 12);
    #if DEBUG
      Serial.print("Device 0 Resolution: ");
      Serial.print(sensors.getResolution(insideThermometer), DEC); 
      Serial.println();
      serialFlush();
    #endif
  #endif
    
  reportCount = REPORT_EVERY;     // report right away for easy debugging
  scheduler.timer(MEASURE, 0);    // start the measurement loop going
}

void loop () {
    loopcounter = loopcounter + 1;
    #if DEBUG
       Serial.print("\n");
       Serial.print((int) loopcounter); 
       Serial.print(": Startloop");
       serialFlush();
    #endif   
        

    #if PIR_PORT
        #if DEBUG
          Serial.print("\n   PIR.triggercheck");
          serialFlush();
        #endif
        if (pir.triggered()) {
//            #if DEBUG
//              Serial.print("\n   PIR.state");
//              serialFlush();
//            #endif
            payload.moved = pir.state();
//            #if DEBUG
//              Serial.print("\n   PIR.doTrigger");
//              serialFlush();
//            #endif
            doTrigger();
        }
    #endif
    
    #if DEBUG
       Serial.print("\n   Waitloop");
       serialFlush();
    #endif     
    switch (scheduler.pollWaiting()) {

        case MEASURE:
            // reschedule these measurements periodically
            scheduler.timer(MEASURE, MEASURE_PERIOD);
            
            #if DEBUG
              Serial.print("\n   doMeasure");
              serialFlush();
            #endif
            
            doMeasure();

            // every so often, a report needs to be sent out
            if (++reportCount >= REPORT_EVERY) {
                reportCount = 0;
                scheduler.timer(REPORT, 0);
            }
            break;
            
        case REPORT:

            #if DEBUG
              Serial.print("\n   doReport");
              serialFlush();
            #endif
            
            doReport();
            break;
    }
}
