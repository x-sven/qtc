#include "HX711-multi.h"
#include "MovingAverage.h"

// Pins to the load cell amp
#define CLK 14      // clock pin to the load cell amp
#define DOUT1 12    // data pin to the first lca
#define DOUT2 10    // data pin to the second lca

#define BOOT_MESSAGE "MIT_HX711_SCALES V1.0"

#define TARE_TIMEOUT_SECONDS 4

byte DOUTS[2] = {DOUT1, DOUT2};

#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)
#define SCALE_FACTOR_1 167000./196.
#define SCALE_FACTOR_2 645400./196.


long int results[CHANNEL_COUNT];
float SCALES[2] = {SCALE_FACTOR_1, SCALE_FACTOR_2};
MovingAverage <long int, 16> results_avg[CHANNEL_COUNT];

struct
{
  long int sum;
  long int count;
  byte size = CHANNEL_COUNT;
} average[CHANNEL_COUNT];

HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);

void setup() {
  Serial.begin(115200);
  Serial.println(BOOT_MESSAGE);
  
  for (int i=0; i<CHANNEL_COUNT; ++i) {
    results[i] = 0;
  }
}


void tare() {
  bool tareSuccessful = false;

  Serial.print("Tare...");
  unsigned long tareStartTime = millis();
  while (!tareSuccessful && millis()<(tareStartTime+TARE_TIMEOUT_SECONDS*1000)) {
    tareSuccessful = scales.tare(20,10000);  //tare(times, tolerance) + reject 'tare' if still ringing
  }
  Serial.println("done!");
}

void printData() {
  scales.read(results);
  Serial.print(millis());
  Serial.print( ":\t");
  for (int i=0; i<scales.get_count(); ++i) {
    results_avg[i].add(results[i]);
    Serial.print( results[i]/SCALES[i],1);  //???
    Serial.print( "\t"); 
    Serial.print( results_avg[i].get()/SCALES[i],1);  
    Serial.print( (i!=scales.get_count()-1)?"\t":"\n");
  }  
}

void loop() {
  
  printData(); //this is for sending raw data, for where everything else is done in processing

  //on serial data (any data) re-tare
  if (Serial.available()>0) {
    while (Serial.available()) {
      char cmd = Serial.read();
        if (cmd == 't'){
        tare();
        }
        else if (cmd == 's'){
          SCALES[0] = 1.;
          SCALES[1] = 1.;
        }
        else if (cmd == 'r'){
          SCALES[0] = SCALE_FACTOR_1;
          SCALES[1] = SCALE_FACTOR_2;
        }
        else {}
    }
  }
 
}