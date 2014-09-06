/*

 Arduino Controlled Passive Infrared Motion Sensor
 
 LemonSlice7 - Instructables.com
 
 */
 
#include <JeeLib.h>

int calibrationTime = 60; 
#define RETRY_PERIOD    30  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     10   // maximum number of times to retry
#define ACK_TIME        20  // number of milliseconds to wait for an ack
#define RETRANSMIT_EVERY 30 // number of seconds to retransmit the sensor state

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2



boolean sensorActive = false;
boolean previousSensorState = false;

static byte myNodeID;    

int pirPin = 7;    //the digital pin connected to the PIR sensor's output
volatile uint32_t last_event;

struct {
    byte moved : 1;  // motion detector: 0..1
} payload;

static void serialFlush () {
    #if ARDUINO >= 100
        Serial.flush();
    #endif  
    delay(2); // make sure tx buf is empty before going back to sleep
}

static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
            return 1;
    }
    return 0;
}

// setup phase
void setup(){
  myNodeID = rf12_config();
  Serial.begin(9600);
  pinMode(pirPin, INPUT);

  //give the sensor some time to calibrate
  Serial.println("Sensor Calibration in Progress");
  Serial.println("------------------------------");
  for(int i = 0; i < calibrationTime; i++){
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println("Sensor Calibration Completed");
  Serial.println("Sensor Reading Active");
  delay(50);
  sensorActive = false;
  previousSensorState = false;
  last_event = millis();
}

// loop sequence
void loop()
{
  // takes the pin value and saves it to the sensorActive boolean value
  if(digitalRead(pirPin) == HIGH)
  {
    sensorActive = true;
  }
  if(digitalRead(pirPin) == LOW)
  {
    sensorActive = false;
  }
  
  // performs action if the state of the sensor changes
  // since this is a loop, here is now it works:
  // if the sensor pin goes HIGH (on) after it being LOW (off), the sensorActive value changes from the previousSensorState value.
  // it then turns on the LED. when the pin goes LOW (off) it will do the same thing but opposite values.
  // it also prints status to serial. it will print the time of triggering by providing the number of seconds that have passed since the program started.
  if((sensorActive != previousSensorState) || ((millis() - last_event) >= (RETRANSMIT_EVERY * 1000)))
  {
    if(sensorActive == true)
    {
      last_event = millis();
      previousSensorState = sensorActive;
      serialFlush();
      Serial.println("---");
      Serial.print("Motion Detected At: ");
      Serial.print(millis()/1000);
      Serial.println(" Seconds");
      serialFlush(); 
      payload.moved = 1;
      for (byte i = 0; i < RETRY_LIMIT; ++i) {
        rf12_sleep(RF12_WAKEUP);
        rf12_sendNow(RF12_HDR_ACK, &payload, sizeof payload);
        rf12_sendWait(RADIO_SYNC_MODE);
//        serialFlush();
        Serial.print(" waitForAck ");
//        serialFlush();
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);
        if (acked) {
          serialFlush();
          Serial.print(" ack\n");
          serialFlush();
          return;
        }
        delay(RETRY_PERIOD * 100);
      } 
      serialFlush();
    }
    if((sensorActive == false) && (sensorActive != previousSensorState))
    {
      previousSensorState = sensorActive;
      last_event = millis();
      serialFlush();
      Serial.println("---");
      Serial.print("Motion Stopped At: ");
      Serial.print(millis()/1000);
      Serial.println(" Seconds"); 
      serialFlush();
      payload.moved = 0;
      for (byte i = 0; i < RETRY_LIMIT; ++i) {
        rf12_sleep(RF12_WAKEUP);
        rf12_sendNow(RF12_HDR_ACK, &payload, sizeof payload);
        rf12_sendWait(RADIO_SYNC_MODE);
        Serial.print(" waitForAck ");
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);
        if (acked) {
          Serial.print(" ack\n");
          serialFlush();
          return;
        }
        delay(RETRY_PERIOD * 100);
      }
      serialFlush();
    }
  }
}




