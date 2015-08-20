//------------------------------------------------------------------------------------------------------------------------------------------------
// kaaNet base Node schetch

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

struct {
     byte setrelay1;
     byte setrelay2;
} payload;

#define RELAYPIN1             6
#define RELAYPIN2             7

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

static void configDump() {
  DEBUG("\r\nNODEID:");DEBUGln(CONFIG.nodeID);
  DEBUG("NETWORKID:");DEBUGln(CONFIG.networkID);
  DEBUG("EncryptKey:");DEBUGln(CONFIG.encryptionKey);
  DEBUG("isHW:");DEBUGln(CONFIG.isHW);  
}

void setup()
{
  pinMode(RELAYPIN1, OUTPUT);
  pinMode(RELAYPIN2, OUTPUT);

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

  DEBUGln("[kaaNet_Base_Node_RFM69.1]");
  digitalWrite(RELAYPIN1, HIGH);
  digitalWrite(RELAYPIN2, HIGH);
  delay(1000);
  digitalWrite(RELAYPIN1, LOW);
  digitalWrite(RELAYPIN2, LOW);
  payload.setrelay1 = 0;
  payload.setrelay2 = 0;
}

//--------------------------------------------------------------------------------------------
// Loop
//--------------------------------------------------------------------------------------------
void loop()
{
  if (radio.receiveDone())
  {
      
      int node_id = (radio.TARGETID);
      DEBUG("rcv: "); DEBUG(radio.TARGETID);DEBUG("> ");DEBUG(radio.DATA[0]);DEBUG(",");DEBUG(radio.DATA[1]);DEBUG(",");DEBUG(radio.DATA[2]);DEBUG(",");DEBUG(radio.DATA[3]);DEBUG(": ");DEBUGln(radio.PAYLOADLEN);
      if ((node_id == CONFIG.nodeID) && (radio.DATA[0] == 23) && radio.PAYLOADLEN == 6) //targeted packet type 23 [legacy 2-relaynode]
      {
        if(radio.DATA[1] != 3) {
	  digitalWrite(RELAYPIN1,radio.DATA[1]);
	  payload.setrelay1 = radio.DATA[1];
        }
        if (radio.DATA[2] != 3) {
	  digitalWrite(RELAYPIN2,radio.DATA[2]);
	  payload.setrelay2 = radio.DATA[2];
	}
        if (radio.ACKRequested()){
          DEBUGln("Gateway requested confirmation of payload received");
          radio.sendACK();
          DEBUGln("ACK sent to Gateway");
        }
//        if (radio.canSend()){
          DEBUG("snd: ");DEBUGln(GATEWAYID); 
          delay(3);
          if (radio.sendWithRetry(GATEWAYID, &payload, sizeof payload,4)){
            DEBUGln("Gateway confirmed Status Echo");  //echo relay status
          } else {
                   DEBUGln("Gateway did NOT confirm Status Echo");
                  }
//	}
        DEBUG("Relay1: ");
	DEBUG(int(payload.setrelay1));
	DEBUG(", Relay2: ");
	DEBUGln(int(payload.setrelay2));
      }
    }    
}
