//------------------------------------------------------------------------------------------------------------------------------------------------
// kaaNet base Node schetch


#define MAX_TIME_TO_RECEIVE 600 // number of seconds to wait for first reception of time and usage data after reset takes place

#include <avr/wdt.h>
#include <EEPROMex.h>      //get it here: http://playground.arduino.cc/Code/EEPROMex
#include <RFM69.h>     //get it here: http://github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash

#define NODEID        3    //unique for each node on same network
#define GATEWAYID     1  //assumed 1 in general
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_868MHZ
#define ENCRYPTKEY    "df39ea10cc412@1k" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    0 //set to 1 only for RFM69HW! Leave 0 if you have RFM69(C)W!
bool promiscuousMode = true; //set to 'true' to sniff all packets on the same network

struct configuration {
  byte frequency;
  byte isHW;
  byte nodeID;
  byte networkID;
  char encryptionKey[16];
  byte separator1;          //separators needed to keep strings from overlapping
  char description[10];
  byte separator2;
} CONFIG;

#define ACK_TIME      30 // max # of ms to wait for an ack
//#define SERIAL_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    57600
#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); Serial.flush();delay(1);}
  #define DEBUGln(input) {Serial.println(input); Serial.flush();delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

RFM69 radio;


//---------------------------------------------------
// Data structures for transfering data between units
//---------------------------------------------------
// typedef struct { byte packettype; int power1; } PayloadTX;         // neat way of packaging data for RF comms
// PayloadTX emontx;

static void configDump() {
  DEBUG("\r\nNODEID:");DEBUGln(CONFIG.nodeID);
  DEBUG("NETWORKID:");DEBUGln(CONFIG.networkID);
  DEBUG("EncryptKey:");DEBUGln(CONFIG.encryptionKey);
  DEBUG("isHW:");DEBUGln(CONFIG.isHW);  
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
    CONFIG.separator1=CONFIG.separator2=0;
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
  radio.promiscuous(promiscuousMode);

  delay(100);				   //wait for RF to settle befor turning on display

  DEBUGln("[kaaNet_Base_Node_RFM69.1]");
}

//--------------------------------------------------------------------------------------------
// Loop
//--------------------------------------------------------------------------------------------
void loop()
{
  if (radio.receiveDone())
  {
      int node_id = (radio.TARGETID);
      DEBUG("rcv: "); DEBUG(radio.TARGETID);DEBUG("> ");DEBUG(radio.DATA[0]);DEBUG(",");DEBUG(radio.DATA[1]);DEBUG(",");DEBUG(radio.DATA[2]);DEBUG(": ");DEBUGln(radio.PAYLOADLEN);
      if ((node_id == 255) && (radio.DATA[0] == 22) && radio.PAYLOADLEN == 7) //broadcast packet type 22 [time]
      {
        RTC.adjust(DateTime(2014, 5, 16, radio.DATA[1], radio.DATA[2], radio.DATA[3]));
        last_emonbase = millis();
        last_time_update = millis();
      }
      if ((node_id == 255) && (radio.DATA[0] == 21) && radio.PAYLOADLEN == 6) //broadcast packet type 21 [verbruik]
      { DEBUGln("display"); emontx = *(PayloadTX*) radio.DATA; 
        last_emontx = millis();
        last_usage_update = millis();
      } 
    }
    
    radio.send(GATEWAYID, &emonglcd, sizeof emonglcd);                     
    
  }
}
