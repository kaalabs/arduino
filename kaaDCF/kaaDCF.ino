#include <DCF77.h>       //https://github.com/thijse/Arduino-Libraries/downloads
#include <Time.h>        //http://www.arduino.cc/playground/Code/Time
#include <Timezone.h>    //https://github.com/JChristensen/Timezone

#define DCF_PIN 3	         // Connection pin to DCF 77 device
#define DCF_INTERRUPT 1		 // Interrupt number associated with pin

//United Kingdom (London, Belfast)
TimeChangeRule rBST = {"BST", Last, Sun, Mar, 1, 60};        //British Summer Time
TimeChangeRule rGMT = {"GMT", Last, Sun, Oct, 2, 0};         //Standard Time
Timezone UK(rBST, rGMT);

//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);

time_t prevDisplay = 0;          // when the digital clock was displayed
time_t time;
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT);

void setup() {
  Serial.begin(115200); 
  DCF.Start();
  setSyncInterval(30);
  setSyncProvider(getDCFTime);
  // It is also possible to directly use DCF.getTime, but this function gives a bit of feedback
  //setSyncProvider(DCF.getTime);

  Serial.println("Waiting for DCF77 UK local time ... ");
  Serial.println("It will take at least 2 minutes until a first update can be processed.");
  while(timeStatus()== timeNotSet) { 
     // wait until the time is set by the sync provider     
     Serial.print(".");
     delay(2000);
  }
}

void loop()
{  
  if( now() != prevDisplay) //update the display only if the time has changed
  {
    prevDisplay = now();
    digitalClockDisplay();  
  }
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.println("");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

unsigned long getDCFTime()
{ 
  time_t DCFtime = DCF.getUTCTime(); // Get  UTC time
  
  if (DCFtime!=0) {
    time_t LocalTime = CE.toLocal(DCFtime);  //Convert to UK time
    return LocalTime;
  }
  return 0;
}
