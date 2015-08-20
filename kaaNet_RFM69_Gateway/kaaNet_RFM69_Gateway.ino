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
#define ENCRYPTKEY    "df39ea10cc412@1k" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    0 //set to 1 only for RFM69HW! Leave 0 if you have RFM69(C)W!
#define RFM69_MAXDATA 61
#define ACK_TIME      30 // max # of ms to wait for an ack
#define LED_PIN     9       // activity LED, comment out to disable
#define VERSION "\n[kaaNet_Gateway_RFM69.2]\n"

#define SERIAL_EN             //comment this out when deploying to an installed SM to save a few KB of sketch size
#define SERIAL_BAUD    57600

#define LED           9 // Moteinos have LEDs on D9

RFM69 radio;

bool promiscuousMode = true; //set to 'true' to sniff all packets on the same network
bool collectMode = false; //set to 'true' only collect data and not respond to ACK_REQ

static char cmd;
static word value;
static byte stack[RFM69_MAXDATA+4], top, sendLen, dest;

struct configuration {
  byte frequency;
  byte isHW;
  byte nodeID;
  byte networkID;
  byte collectMode;                  //separators needed to keep strings from overlapping 
  byte promiscuousMode;              //separators needed to keep strings from overlapping
  char encryptionKey[16];
  byte separator1;
  char description[10];
  byte separator2;
} CONFIG;

static void printOneChar (char c) {
    Serial.print(c);
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

static void displayVersion () {
    showString(PSTR(VERSION));
}

static void rfm69_writeConfig(){
 radio.sleep();
 delay(3);
 EEPROM.writeBlock(0, CONFIG);
}
static void initRadio(){
  
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
  promiscuousMode = (CONFIG.promiscuousMode == 1);
  radio.promiscuous(promiscuousMode);
  collectMode = (CONFIG.collectMode == 1);
}

static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, on);
#endif
}

static void showWord (unsigned int value) {
        Serial.print((word) value);    
}


static void showByte (byte value) {
   Serial.print((word) value); // Serial.print causes issues with my terminal emulator (PuTTY)
                                    // showByte(0x0A) produced a LF etc rather than 10
}


static void configDump() {
  displayVersion();
  EEPROM.readBlock(0, CONFIG);
  showString(PSTR("Current coniguration:\n"));
  showString(PSTR("NODEID:"));showByte(CONFIG.nodeID);Serial.println();
  showString(PSTR("NETWORKID:"));showByte(CONFIG.networkID);Serial.println();
  showString(PSTR("Description:"));Serial.print(CONFIG.description);Serial.println();
  showString(PSTR("Frequency:"));Serial.print(CONFIG.frequency==RF69_433MHZ?"433":CONFIG.frequency==RF69_868MHZ?"868":"915");showString(PSTR(" mhz"));Serial.println();
  showString(PSTR("EncryptKey:"));Serial.print(CONFIG.encryptionKey);Serial.println();
  showString(PSTR("isHW:"));showByte(CONFIG.isHW);Serial.println();
  showString(PSTR("collectMode:"));showByte(CONFIG.collectMode);Serial.println();
  showString(PSTR("promiscuousMode:"));showByte(CONFIG.promiscuousMode);Serial.println();
  Serial.println();
}  
const char helpText1[] PROGMEM =
    "\n"
    "Available commands:\n"
    " <nnn>i      - set node ID (standard node ids are 1..127)\n"
    " <n>b       - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
    " <n>c       - set collect mode (0 = OFF (default), 1 = ON (only collect data, do not respond)\n"
    " <nnn>g     - set Network ID (0..127)\n"
    " <n>p       - set promiscuous mode (0 = OFF, 1 = ON (default, receive all network trafic data)\n"
    " <nnn>g     - set network group (RFM12 only allows 212, 0 = any)\n"
    " ...,<nn>a  - send data packet to node <nn>, request ack with retry\n"
    " ...,<nn>s  - send data packet to node <nn>, no ack\n"
    " ... <nn>   - Space character is a valid delimiter\n"
    " h          - Show this help text\n"

;

static void showHelp () {
    showString(helpText1);
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
        printOneChar(c);
    }
    if (c > ' ') {  
        switch (c) {
          case 'h': //show help text
            showHelp();
            break;
          case 'a': // send packet to node ID N, request an ack
          case 's': // send packet to node ID N, no ack
            cmd = c;
            sendLen = top;
            dest = value;
            break;
          case 'c': //collectMode
            CONFIG.collectMode = value;
            collectMode = (CONFIG.collectMode == 1);
            rfm69_writeConfig();
            configDump();
            break;
          case 'i': //set nodeID
            CONFIG.nodeID = value;
            rfm69_writeConfig();
            initRadio();
            break;
          case 'b': //set mhz Band
            switch(value) {
              case 4: CONFIG.frequency = RF69_433MHZ; rfm69_writeConfig();initRadio();break;
              case 8: CONFIG.frequency = RF69_868MHZ; rfm69_writeConfig();initRadio();break;
              case 9: CONFIG.frequency = RF69_915MHZ; rfm69_writeConfig();initRadio();break;
            }
            break;
          case 'g': //set nodeID
            CONFIG.networkID = value;
            rfm69_writeConfig();
            initRadio();
            break;
          case 'p': //promiscuousMode
            CONFIG.promiscuousMode = value;
            promiscuousMode = (CONFIG.promiscuousMode == 1);
            radio.promiscuous(promiscuousMode);
            rfm69_writeConfig();
            configDump();
            break;  
          default:
            showString(PSTR("key="));showByte(c);Serial.println();
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
    showString(PSTR("No valid config found in EEPROM, writing defaults"));
      CONFIG.separator2=CONFIG.separator1=0;
      CONFIG.collectMode=0;
      CONFIG.promiscuousMode=1;
      CONFIG.frequency=FREQUENCY;
      CONFIG.description[0]=0;
      strcpy(CONFIG.encryptionKey, ENCRYPTKEY);
      CONFIG.isHW=IS_RFM69HW;
      CONFIG.nodeID=NODEID;
      CONFIG.networkID=NETWORKID;
      rfm69_writeConfig();
  }
  
  initRadio();
}

byte ackCount=0;

void loop() {
   
if (Serial.available())
        handleInput(Serial.read());
  if (radio.receiveDone())
  {
    showString(PSTR("OK "));
    showByte(radio.SENDERID);
    printOneChar(' ');
    if (promiscuousMode)
    {
      showByte(radio.TARGETID);
      printOneChar(' ');
    }
    for (byte i = 0; i < radio.DATALEN; i++){
      showByte(radio.DATA[i]);
      printOneChar(' ');
    }
    printOneChar('(');
    Serial.print(radio.RSSI);
    printOneChar(')');
    
    if (radio.ACKRequested() && !collectMode)
    {
      byte theNodeID = radio.SENDERID;
      showString(PSTR("\n <- ACK_REQ "));showByte(theNodeID);
      radio.sendACK();
      showString(PSTR("\n -> ACK "));showByte(theNodeID);
      // When a node requests an ACK, respond to the ACK
    }
    Serial.println();
  }
  if (cmd) {
            showString(PSTR(" -> "));
            showByte(sendLen);
            showString(PSTR(" b\n"));
            byte header = cmd == 'a' ? 1 : 0;
            if (header == 0) {
              if (dest) {
                 //showString(PSTR(" send with DEST\n"));
                 activityLed(1);
                 radio.send(dest, stack, sendLen);
                 activityLed(0);
              } else {
                         //showString(PSTR(" send broadcast\n"));
                         activityLed(1);
                         radio.send(RF69_BROADCAST_ADDR, stack, sendLen);
                         activityLed(0);
                      }
            } else {
                      if (dest) {
                                   showString(PSTR(" -> ACK_REQ ")); 
                                   showByte(dest); 
                                   Serial.println();
                                   activityLed(1);
                                   if (radio.sendWithRetry(dest, stack, sendLen)){
                                     showString(PSTR(" <- ACK "));
                                     showByte(dest);
                                     Serial.println();
                                   } else {
                                       showString(PSTR("! ACK_REQ "));
                                       showByte(dest);
                                       showString(PSTR(" FAIL\n"));
                                     }
                                   activityLed(0);  
                      } else {
                         showString(PSTR(" -> ACK_REQ <broadcast>\n")); //is it possible to request an ACK for a broadcast?
                         activityLed(1);
                         if (radio.sendWithRetry(RF69_BROADCAST_ADDR, stack, sendLen)){
                           showString(PSTR(" <- ACK "));
                           showByte(dest);
                           Serial.println();
                          } else {
                              showString(PSTR(" ! ACK_REQ <broadcast> FAIL\n"));
                            }
                          activityLed(0);
                        }
              }
            cmd = 0;
  }    
}
