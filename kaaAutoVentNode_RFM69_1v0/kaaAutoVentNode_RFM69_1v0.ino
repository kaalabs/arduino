//------------------------------------------------------------------------------------------------------------------------------------------------
// kaaNet base Node schetch


#define MAX_TIME_TO_RECEIVE 600 // number of seconds to wait for first reception of time and usage data after reset takes place

#include <avr/wdt.h>
#include <EEPROMex.h>      //get it here: http://playground.arduino.cc/Code/EEPROMex
#include <RFM69.h>     //get it here: http://github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash

#define NODEID        99    //unique for each node on same network
#define GATEWAYID     1  //assumed 1 in general
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_868MHZ
#define ENCRYPTKEY    "df39ea10cc412@1k" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    1 //set to 1 only for RFM69HW! Leave 0 if you have RFM69(C)W!
bool promiscuousMode = true; //set to 'true' to sniff all packets on the same network

#define RELAYPIN1             6
#define RELAYPIN2             7

#define ACK_TIME      30 // max # of ms to wait for an ack
#define SERIAL_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    57600
#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); Serial.flush();delay(1);}
  #define DEBUGln(input) {Serial.println(input); Serial.flush();delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

RFM69 radio;

byte memLastHumidityReading = 0;


//---------------------------------------------------
// Data structures for transfering data between units
//---------------------------------------------------
typedef struct { byte packettype; } AskforConfig;         // neat way of packaging data for RF comms
AskforConfig AC;

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

typedef struct {                                                     
  	  int temp;
          int temp_external;
          int humidity;    
          int battery;          	                                      
} HumiTempPayload;

HumiTempPayload emonth;

typedef struct {
  byte modus;// [0..1] (0=DELTA TRIGGERS, 1= FIXED TRIGGER [default: 0]
  byte triggeronsetpoint; //BYTE [0..100]% (DELTA TRIGGERS = Ignore, 0 = OFF) [default: 0]
  byte triggeroffsetpoint; //[0..100]% (DELTA TRIGGERS = Ignore, 0 = OFF) [default: 0]
  byte timelimit; //[0..255] min. (0=UNLIMITED) [default: 120]
  byte hysteresistime; //BYTE[0..255] min. (0=None) [default: 5]
  byte autoondeltatrigger; //[5..50]%  [default: 10]    
  byte autooffdeltatrigger; //[5..50]% [default: 5]
  byte huminodeid; //nodeID of emonTH node which sends humidity
} ventnodesetup;

ventnodesetup NodeSetup;


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

  DEBUGln("[kaaAutoVentNode_RFM69.1]");
  
  NodeSetup.modus = 0;
  NodeSetup.triggeronsetpoint = 0;
  NodeSetup.triggeroffsetpoint = 0;
  NodeSetup.timelimit = 120;
  NodeSetup.hysteresistime = 5;
  NodeSetup.autoondeltatrigger = 10;
  NodeSetup.autooffdeltatrigger = 5;
  NodeSetup.huminodeid = GATEWAYID;
  
  AC.packettype = 99;
  radio.send(GATEWAYID, &AC, sizeof AC);
  pinMode(RELAYPIN1, OUTPUT);
  pinMode(RELAYPIN2, OUTPUT);
  digitalWrite(RELAYPIN1, HIGH);
  digitalWrite(RELAYPIN2, HIGH);
}

//--------------------------------------------------------------------------------------------
// Loop
//--------------------------------------------------------------------------------------------
void loop()
{
  if (radio.receiveDone())
  {
      int node_id = (radio.TARGETID);
      if (node_id == CONFIG.nodeID){
        if (radio.DATA[0] == 99){ //configure individual parameters AutoVentNode
          int configItem = radio.DATA[1];
          switch (configItem) {
            case 1: NodeSetup.modus = radio.DATA[2];
                    break; // 01.Modus BYTE [0..1] (0=DELTA TRIGGERS, 1= FIXED TRIGGER [default: 0]
            case 2: NodeSetup.triggeronsetpoint = radio.DATA[2];
                    break; // 02.TRIGGER ON Setpoint BYTE [0..100]% (DELTA TRIGGERS = Ignore, 0 = OFF) [default: 0]
            case 3: NodeSetup.triggeroffsetpoint = radio.DATA[2];
                    break; // 03.TRIGGER OFF Setpoint BYTE [0..100]% (DELTA TRIGGERS = Ignore, 0 = OFF) [default: 0]
            case 4: NodeSetup.timelimit = radio.DATA[2];
                    break; // 04.Zet Tijdlimiet BYTE [0..255] min. (0=UNLIMITED) [default: 120]
            case 5: NodeSetup.hysteresistime = radio.DATA[2];
                    break; // 05.Zet Hysteresis tijd BYTE[0..255] min. (0=None) [default: 5]
            case 6: NodeSetup.autoondeltatrigger = radio.DATA[2];
                    break; // 06.AUTO ON Delta Trigger [5..50]%  [default: 10]
            case 7: NodeSetup.autooffdeltatrigger = radio.DATA[2];
                    break; // 07.AUTO OFF Delta Trigger [5..50]% [default: 5]
            case 8: break; // 08.Set of configuration parameters for DELTA TRIGGER operation [default: 01.02, 04.120, 05.05, 06.10, 07.05]
            case 9: break; // 09.Set of configuration parameters for FIXED TRIGGER operation [default: 01.01, 02.70, 03.60, 04.120, 05.05]
            case 10: NodeSetup.huminodeid = radio.DATA[2];
                     break; //10.Set HumiNodeID
            default: break;
          }
        }
        if (radio.DATA[0] == 99){}
      }
  }
  switch (NodeSetup.modus) {
    case 0: break; //0 = DELTA TRIGGERS
    case 1: break; //1 = FIXED TRIGEGER
  }
}
