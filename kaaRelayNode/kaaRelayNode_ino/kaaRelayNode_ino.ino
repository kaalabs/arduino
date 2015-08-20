/// Small demo for the Relay Plug, receives wireless packets and sets relays.
// 2010-07-05 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// 2013-10-01 <remco@remcokortekaas.nl> fork relay_demo

#define RF69_COMPAT 0

#include <JeeLib.h>

#define RADIO_SYNC_MODE 2

Port relays (1);

struct {
     byte setrelay1;
     byte setrelay2;
     byte dest;
} payload;

void setup () {
    rf12_configSilent();
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
    if (rf12_crc == 0) if ((rf12_hdr & RF12_HDR_DST) == RF12_HDR_DST) // no rf errors, ignore broadcasts 
    {
        relays.digiWrite(rf12_data[0]);
        relays.digiWrite2(rf12_data[1]);
        payload.setrelay1 = rf12_data[0];
        payload.setrelay2 = rf12_data[1];
        rf12_sendNow(0, &payload, sizeof payload);  //echo relay status
        rf12_sendWait(RADIO_SYNC_MODE);
    }
  }
}
