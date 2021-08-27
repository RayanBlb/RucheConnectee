
#include "Adafruit_CCS811.h"
#include "ExtVarGlobales.h"
#include "Definition.h"


Adafruit_CCS811 ccs;

void setup_CO2() {

  
  if(!ccs.begin(0x5B)){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }
  //calibrate temperature sensor
  while(!ccs.available());
  temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
}

String read_CO2_value() {
  if(ccs.available()){
    temp = ccs.calculateTemperature();
    if(!ccs.readData()){
      ChaineCO2 = "CO2: "+String(ccs.geteCO2())+"ppm, TVOC: "+String(ccs.getTVOC())+"ppb   Temp:"+String(temp);
      //Serial.println("CO2: "+String(ccs.geteCO2())+"ppm, TVOC: "+String(ccs.getTVOC())+"ppb   Temp:"+String(temp));
    }
    else{
      Serial.println("Sensor read ERROR!");
      ccs.readData();
    }
  }
  return ChaineCO2;
}
