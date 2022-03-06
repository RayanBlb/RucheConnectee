#include "definition.h"
#include "donnees.h"
#include "LoRa.h"
#include "ai_functions.h"
#include "audio_functions.h"
#include <LITTLEFS.h>

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
  
  uint16_t hygro = (payload[17] * 256u) + payload[18];
  uint16_t v_bat = (payload[19] * 256u) + payload[20];
  uint16_t charg_bat = (payload[21] * 256u) + payload[22];
  uint16_t IA = (payload[23] * 256u) + payload[24];
  
  Serial.printf("Trame after cast : %C | %X:%X:%X:%X:%X:%X | %u | %u | %u | %u | %u | %u | %u | %u | %u\n",payload[0],payload[1],payload[2],payload[3],payload[4],payload[5],payload[6],son,temperature,CO2,CO2_TVOC,Piezo,hygro,v_bat,charg_bat,IA);
  
}

void loop() {
  Serial.printf("test : %s",ai());
  //build_trame(payload);
  //debug_trame();
  //lora_send_trame(payload);
}
