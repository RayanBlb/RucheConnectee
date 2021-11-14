#include "definition.h"
#include "donnees.h"
#include "LoRa.h"

//Trame
uint8_t payload[PAYLOAD_LEN];

void setup() {

  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,HIGH);
  
  SPI.begin(SCK, MISO, MOSI, CS);
  LoRa.setPins(SS, RST, DI0);
  
  Serial.begin(9600);
}

void debug_trame(){
  
  //Trame forme : I|macAdresse|son|temperature|CO2|TVOC|Piezo|code_erreur

  /* I = payload[0] | 1 * uint8_t
  macAdresse = payload[1] to payload[6] | 6 * uint8_t
  son = payload[7] et payload[8] | 1 * uint8_t
  temperature = payload[9] | 1 * uint8_t
  CO2 = payload[10] et payload[11] | 2 * uint8_t
  CO2_TVOC = payload[12] et payload[13] | 2 * uint8_t
  Piezo = payload[14] et payload[15] | 2 * uint8_t
  code_erreur = 1 * uint8_t <-- information transmises via la fonction "LoRa_send_payload" */

  build_trame(payload);

  uint16_t son = (payload[7] * 256u) + payload[8];
  uint16_t CO2 = (payload[10] * 256u) + payload[11];
  uint16_t CO2_TVOC = (payload[12] * 256u) + payload[13];
  uint16_t Piezo = (payload[14] * 256u) + payload[15];
  
  Serial.printf("Trame after cast : %C | %X:%X:%X:%X:%X:%X | %u | %u | %u | %u | %u | %d\n",payload[0],payload[1],payload[2],payload[3],payload[4],payload[5],payload[6],son,payload[9],CO2,CO2_TVOC,Piezo);
  
}

void loop() {
  build_trame(payload);
  debug_trame();
  lora_send_trame(payload);
}
