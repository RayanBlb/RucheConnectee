#include "Adafruit_CCS811.h" //librerie du capteur CO2
#include "arduinoFFT.h"
#include "WiFi.h"

#include "definition.h"

//Variable de température
float temperature;

//Structure données
struct data_trame { 
  char type; 
  uint8_t mac[6];
  double son;
  float temperature;
  int CO2;
  int CO2_TVOC;
  double Piezo;
  int erreur;
} ds;

uint8_t read_mac(uint8_t *mac){
  esp_efuse_read_mac(mac);
  //Serial.printf("mac : %02X:%02X:%02X:%02X:%02X:%02X \n",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return *mac;
}

uint8_t build_trame(uint8_t *payload){
  
  ds.type = 'I';
  read_mac(ds.mac);
  ds.son = 101;
  ds.temperature = 24;
  ds.CO2 = 486;
  ds.CO2_TVOC = 22;
  ds.Piezo = 50;
  ds.erreur = 0;

  Serial.printf("Trame before cast : %c | %X:%X:%X:%X:%X:%X | %f | %f | %d | %d | %f | %d\n",ds.type,ds.mac[0],ds.mac[1],ds.mac[2],ds.mac[3],ds.mac[4],ds.mac[5],ds.son,ds.temperature,ds.CO2,ds.CO2_TVOC,ds.Piezo,ds.erreur);
  
  payload[0] = uint8_t(ds.type);

  for (int i = 1; i<7; i++){
    payload[i] = uint8_t(ds.mac[i-1]);
  }
  
  payload[7] = uint8_t((uint16_t(ds.son) & 0xFF00) >> 8);
  payload[8] = uint8_t((uint16_t(ds.son) & 0x00FF));
  
  payload[9] = uint8_t(ds.temperature);

  payload[10] = uint8_t((uint16_t(ds.CO2) & 0xFF00) >> 8);
  payload[11] = uint8_t((uint16_t(ds.CO2) & 0x00FF));

  payload[12] = uint8_t((uint16_t(ds.CO2_TVOC) & 0xFF00) >> 8);
  payload[13] = uint8_t((uint16_t(ds.CO2_TVOC) & 0x00FF));

  payload[14] = uint8_t((uint16_t(ds.Piezo) & 0xFF00) >> 8);
  payload[15] = uint8_t((uint16_t(ds.Piezo) & 0x00FF));
  
  return *payload;
}
