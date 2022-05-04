#include "donnees.h"
#include "LoRa.h"
#include "config.h"

//Trame
uint8_t payload[PAYLOAD_LEN];

void setup() {

  Serial.begin(9600);

  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,HIGH);

  setup_donnees();

  SPI.begin(SCK, MISO, MOSI);
  LoRa.setPins(SS, RST, DI0);
}

//Fonction debug trame
void debug_trame(){ //Fonction de debug de la trame
  
  //Trame forme : I|macAdresse|son|Temperature|CO2|TVOC|Piezo|hygro|v_bat|charg_bat|IA|Erreur
  
  build_trame(payload);

  char type = payload[0];

  uint16_t son = (payload[7] * 256u) + payload[8];
  uint16_t Temperature = (payload[9] * 256u) + payload[10];
  uint16_t CO2 = (payload[11] * 256u) + payload[12];
  uint16_t CO2_TVOC = (payload[13] * 256u) + payload[14];
  uint16_t Piezo = (payload[15] * 256u) + payload[16];

  uint8_t hygro = payload[17];
  uint16_t v_bat = (payload[18] * 256u) + payload[19];
  uint8_t charg_bat = payload[20];
  uint8_t IA = payload[21];

  
  Serial.printf("Trame after cast : %C | %X:%X:%X:%X:%X:%X | %u | %u | %u | %u | %u || %u | %u | %u | %u\n",type,payload[1],payload[2],payload[3],payload[4],payload[5],payload[6],son,Temperature,CO2,CO2_TVOC,Piezo,hygro,v_bat,charg_bat,IA);
}

//Loop de travail
void loop() {
  //debug_trame();
  build_trame(payload);
  lora_send_trame(payload);
}
