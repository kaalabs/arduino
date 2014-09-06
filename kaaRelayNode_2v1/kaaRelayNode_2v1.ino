/// Small demo for the Relay Plug, receives wireless packets and sets relays.
// 2010-07-05 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// 2013-10-01 <remco@remcokortekaas.nl> fork relay_demo

#define RF69_COMPAT 0

#include <JeeLib.h>

#define RADIO_SYNC_MODE 2

#define MYNODE 3            // Should be unique on network, node ID 30 reserved for base station
#define RF_freq RF12_868MHZ     // frequency - match to same frequency as RFM12B module (change to 868Mhz or 915Mhz if appropriate)
#define group 5

Port relays (1);

struct {
     byte setrelay1;
     byte setrelay2;
} payload;

void setup () {
  delay(500); 				   //wait for power to settle before firing up the RF
  Serial.begin(57600);
  Serial.print("\n[kaaRelaynode.3]");
  delay(100);				   //wait for RF to settle befor turning on display
  rf12_initialize(MYNODE, RF_freq, group);
  rf12_configDump();
  relays.mode(OUTPUT);
  relays.digiWrite(0);
  relays.mode2(OUTPUT);
  relays.digiWrite2(0);
  relays.digiWrite2(1);
  relays.digiWrite(1);
  delay(1000);
  relays.digiWrite(0);
  relays.digiWrite2(0);
  payload.setrelay1 = 0;
  payload.setrelay2 = 0;
  rf12_sendNow(0, &payload, sizeof payload);  //echo relay status
  rf12_sendWait(RADIO_SYNC_MODE);    
}

void loop () {
  if (rf12_recvDone())
  {
    if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)  // clean receive
    {
      int node_id = (rf12_hdr & 0x1F);
      Serial.println(node_id);
      Serial.flush();
      if ((rf12_hdr & RF12_HDR_DST) == RF12_HDR_DST && (node_id == 3) && (rf12_data[0] == 23) && rf12_len == 3) //broadcast packet type 23 [kaaRelay]
      {
        if(rf12_data[1] != 3) {
			relays.digiWrite(rf12_data[1]);
			payload.setrelay1 = rf12_data[1];
		}
        if (rf12_data[2] != 3) {
			relays.digiWrite2(rf12_data[2]);
			payload.setrelay2 = rf12_data[2];
		}
        rf12_sendNow(0, &payload, sizeof payload);  //echo relay status
        rf12_sendWait(RADIO_SYNC_MODE);
		Serial.print("Relay1: ");
		Serial.print(int(payload.setrelay1));
		Serial.print(", Relay2: ");
		Serial.println(int(payload.setrelay2));
		Serial.flush();
      }
    }
  }
}


