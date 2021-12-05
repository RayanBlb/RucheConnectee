#include "definition.h"
#include "donnees.h"
#include "LoRa.h"

//Trame
uint8_t payload[PAYLOAD_LEN];

void setup() {

  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,HIGH);

  setup_donnees();

  SPI.begin(SCK, MISO, MOSI, CS);
  LoRa.setPins(SS, RST, DI0);
  
  Serial.begin(9600);
}

void debug_trame(){ //Fonction de debug de la trame
  
  //Trame forme : I|macAdresse|son|temperature|CO2|TVOC|Piezo
  
  build_trame(payload);

  uint16_t son = (payload[7] * 256u) + payload[8];
  uint16_t temperature = (payload[9] * 256u) + payload[10];
  uint16_t CO2 = (payload[11] * 256u) + payload[12];
  uint16_t CO2_TVOC = (payload[13] * 256u) + payload[14];
  uint16_t Piezo = (payload[15] * 256u) + payload[16];
  
  Serial.printf("Trame after cast : %C | %X:%X:%X:%X:%X:%X | %u | %u | %u | %u | %u\n",payload[0],payload[1],payload[2],payload[3],payload[4],payload[5],payload[6],son,temperature,CO2,CO2_TVOC,Piezo);
  
}

void loop() {
  build_trame(payload);
  //debug_trame();
  lora_send_trame(payload);
}
