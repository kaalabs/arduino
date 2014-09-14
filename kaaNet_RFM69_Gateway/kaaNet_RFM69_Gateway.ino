// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption
// Passes through any wireless received messages to the serial port & responds to ACKs
// It also looks for an onboard FLASH chip, if present
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/

#include <EEPROMex.h>      //get it here: http://playground.arduino.cc/Code/EEPROMex
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

#define NODEID        1    //unique for each node on same network
#define GATEWAYID     1  //assumed 1 in general
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "df39ea10cc412@1k" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    0 //set to 1 only for RFM69HW! Leave 0 if you have RFM69(C)W!
#define RFM69_MAXDATA 61
#define ACK_TIME      30 // max # of ms to wait for an ack
#define LED_PIN     9       // activity LED, comment out to disable

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

RFM69 radio;

bool promiscuousMode = true; //set to 'true' to sniff all packets on the same network
static char cmd;
static word value;
static byte stack[RFM69_MAXDATA+4], top, sendLen, dest; //TODO: RF12_MAXDATA convert to RFM69

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

static void showWord (unsigned int value) {
        Serial.print((word) value);    
}

static void printOneChar (char c) {
    Serial.print(c);
}

static void activityLed (byte on) {    //TODO: convert from jeeLib Port
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
#endif
}

static void showByte (byte value) {
   Serial.print((word) value); // Serial.print causes issues with my terminal emulator (PuTTY)
                               // showByte(0x0A) produced a LF etc rather than 10
}

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            printOneChar('\r');
        printOneChar(c);
    }
}

static void configDump() {
  DEBUG("\r\nNODEID:");DEBUGln(CONFIG.nodeID);
  DEBUG("NETWORKID:");DEBUGln(CONFIG.networkID);
  DEBUG("EncryptKey:");DEBUGln(CONFIG.encryptionKey);
  DEBUG("isHW:");DEBUGln(CONFIG.isHW);  
}

const char helpText1[] PROGMEM =
    "\n"
    "Available commands:\n"
    " <nn>i      - set node ID (standard node ids are 1..30)\n"
    " <n>b       - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
    " <nnn>g     - set network group (RFM12 only allows 212, 0 = any)\n"
    " ...,<nn>a  - send data packet to node <nn>, request ack with retry\n"
    " ...,<nn>s  - send data packet to node <nn>, no ack\n"
    " ... <nn>   - Space character is a valid delimiter\n"

;

static void showHelp () {
    showString(helpText1);
    showString(PSTR("Current configuration:\n"));
    configDump();
}
static void handleInput (char c) {
    //      Variable value is now 16 bits to permit offset command, stack only stores 8 bits
    //      not a problem for offset command but beware.
    if ('0' <= c && c <= '9') {
        value = 10 * value + c - '0';
        return;
    }
    if (c == ',' || c == ' ') {   // Permit comma or space as delimiters
        if (top < sizeof stack)
            stack[top++] = value; // truncated to 8 bits
        value = 0;
        return;
    }
    if (32 > c || c > 'z') {      // Trap unknown characters
            showString(PSTR("Key="));
            showByte(c);          // Highlight Tiny serial framing errors.  
            Serial.println();
            value = top = 0;      // Clear up
    }
    if ('a' <= c && c <= 'z') {
        showString(PSTR("> "));
        for (byte i = 0; i < top; ++i) {
            showByte(stack[i]);
            printOneChar(',');
        }
        showWord(value);
        Serial.println(c);
    }
    if (c > ' ') {  
        switch (c) {
          case 'a': // send packet to node ID N, request an ack
          case 's': // send packet to node ID N, no ack
            cmd = c;
            sendLen = top;
            dest = value;
            break;
          default:
            showHelp();
        } // End case group
        
    }
    value = top = 0;
}

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
}

byte ackCount=0;

void loop() {
   
if (Serial.available())
        handleInput(Serial.read());



  if (radio.receiveDone())
  {
    Serial.print("OK ");Serial.print(radio.SENDERID, DEC);Serial.print(" ");
    if (promiscuousMode)
    {
      Serial.print(radio.TARGETID, DEC);Serial.print(" ");
    }
    for (byte i = 0; i < radio.DATALEN; i++){
      Serial.print((byte)radio.DATA[i]);Serial.print(" ");
    }
    Serial.print("(");Serial.print(radio.RSSI);Serial.print(")");
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      Serial.print("\n\r <- ACK_REQ ");Serial.print(theNodeID);
      radio.sendACK();
      Serial.print("\n\r -> ACK ");Serial.print(theNodeID);
      // When a node requests an ACK, respond to the ACK
    }
    Serial.println();
    Blink(LED,3);
  }
  if (cmd) {
        if (radio.canSend()) {
            activityLed(1);

            showString(PSTR(" -> "));
            showByte(sendLen);
            showString(PSTR(" b\n"));
            byte header = cmd == 'a' ? 1 : 0;
            if (header == 0) {
              if (dest) {
                 //showString(PSTR(" send with DEST\n"));
                 radio.send(dest, stack, sendLen);
              } else {
                         //showString(PSTR(" send broadcast\n"));
                         radio.send(255, stack, sendLen);
                      }
            } else {
                      if (dest) {
                                //   showString(PSTR("send with DEST and ACK\n"));
                                   radio.sendWithRetry(dest, stack, sendLen);
                      } else {
                         // showString(PSTR("send broadcast with ACK\n"));
                         radio.sendWithRetry(255, stack, sendLen);
                        }
              }
            cmd = 0;
            activityLed(0);
        } else {
            showString(PSTR(" Busy\n"));  // Not ready to send
            cmd = 0;                     // Request dropped
          }
  }    
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
