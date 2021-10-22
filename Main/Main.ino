#include "donnees.h"
#include "Definition.h"

String SendLoraChaine;

struct datas { 
  char type; 
  uint8_t mac[6];
  uint16_t son;
  uint16_t temperature;
  uint16_t CO2;
  uint16_t CO2_TVOC;
  uint16_t Piezo;
  uint8_t erreur;
} ds;

void setup() {

  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,HIGH);
  
  Serial.begin(115200);

  setup_donnees();
  
  Serial.begin(9600);
}

void loop() {
  //Trame forme : I+macAdresse+son+temperature+CO2+TVOC+Piezo
  
  ds.type = 'I';
  read_mac(ds.mac);
  ds.son = read_Son();
  ds.temperature = read_Temperature();
  ds.CO2 = read_CO2();
  ds.CO2_TVOC = read_CO2_TVOC();
  ds.Piezo = read_Piezo();
  ds.erreur = 0;

  SendLoraChaine = String(ds.son)+"|"+String(ds.temperature)+"|"+String(ds.CO2)+"|"+String(ds.CO2_TVOC)+"|"+String(ds.Piezo)+"\n";
 
  Serial.printf("%C | %02X:%02X:%02X:%02X:%02X:%02X | %i | %i | %i | %i | %i | %02X \n",ds.type,ds.mac[0], ds.mac[1], ds.mac[2], ds.mac[3], ds.mac[4], ds.mac[5],ds.son,ds.temperature,ds.CO2,ds.CO2_TVOC,ds.Piezo,ds.erreur);
}
