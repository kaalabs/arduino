// Sample RFM69 sender/node sketch, with ACK and optional encryption
// Sends periodic messages of increasing length to gateway (id=1)
// It also looks for an onboard FLASH chip, if present
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/

#include <EEPROMex.h>      //get it here: http://playground.arduino.cc/Code/EEPROMex
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

#define NODEID        2    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "df39ea10cc412@1k" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    0 //set to 1 only for RFM69HW! Leave 0 if you have RFM69(C)W!

#define ACK_TIME      30 // max # of ms to wait for an ack
#define SERIAL_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    57600
#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#define LED           9 // Moteinos have LEDs on D9

int TRANSMITPERIOD = 300; //transmit a packet to gateway so often (in ms)
char payload[] = "123 ABCDEFGHIJKLMNOPQRSTUVWXYZ";

byte sendSize=0;
boolean requestACK = false;


RFM69 radio;

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

void setup() {
  #ifdef SERIAL_EN
    Serial.begin(SERIAL_BAUD);
  #endif
  EEPROM.readBlock(0, CONFIG);
  if (CONFIG.frequency!=RF69_433MHZ && CONFIG.frequency!=RF69_868MHZ && CONFIG.frequency!=RF69_915MHZ) // virgin CONFIG, expected [4,8,9]
  {
    Serial.println("No valid config found in EEPROM, writing defaults");
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
  radio.encrypt(CONFIG.encryptionKey);
  
  if (CONFIG.isHW) radio.setHighPower(); //use with RFM69HW ONLY!
  radio.encrypt(CONFIG.encryptionKey);
  
  radio.writeReg( 0x03, 0x06); //19.2kBps
  radio.writeReg( 0x04, 0x83);
  radio.writeReg( 0x05, 0x01); //default:5khz, (FDEV + BitRate/2 <= 500Khz)
  radio.writeReg( 0x06, 0x9A);
  radio.writeReg( 0x19, 0x40 | 0x10 | 0x03); 
  radio.writeReg( 0x3d, 0xC0 | 0x02 | 0x00); //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
  
  DEBUG("\r\nNODEID:");DEBUGln(CONFIG.nodeID);
  DEBUG("NETWORKID:");DEBUGln(CONFIG.networkID);
  DEBUG("EncryptKey:");DEBUGln(CONFIG.encryptionKey);
  DEBUG("isHW:");DEBUGln(CONFIG.isHW);

  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}

long lastPeriod = -1;

void loop() {
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input >= 48 && input <= 57) //[0,9]
    {
      TRANSMITPERIOD = 100 * (input-48);
      if (TRANSMITPERIOD == 0) TRANSMITPERIOD = 1000;
      Serial.print("\nChanging delay to ");
      Serial.print(TRANSMITPERIOD);
      Serial.println("ms\n");
    }

    if (input == 'r') //d=dump register values
      radio.readAllRegs();
    //if (input == 'E') //E=enable encryption
    //  radio.encrypt(KEY);
    //if (input == 'e') //e=disable encryption
    //  radio.encrypt(null);


  }

  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
    Blink(LED,5);
    Serial.println();
  }


  int currPeriod = millis()/TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    lastPeriod=currPeriod;
    Serial.print("Sending[");
    Serial.print(sendSize);
    Serial.print("]: ");
    for(byte i = 0; i < sendSize; i++)
      Serial.print((char)payload[i]);

    if (radio.sendWithRetry(GATEWAYID, payload, sendSize))
     Serial.print(" ok!");
    else Serial.print(" nothing...");

    sendSize = (sendSize + 1) % 31;
    Serial.println();
    Blink(LED,3);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
