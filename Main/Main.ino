#include "donnees.h"
#include "Definition.h"

String SendLoraChaine;

void setup() {

  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,HIGH);
  
  Serial.begin(115200);

  setup_donnees();
  
  Serial.begin(9600);
}

void loop() {
  //Trame forme : I+macAdresse+son+temperature+CO2+TVOC+Piezo
  SendLoraChaine = "I|"+read_mac()+"|"+String(read_Son())+"|"+String(read_Temperature())+"|"+read_CO2()+"|"+String(read_Piezo());
 Serial.println(SendLoraChaine);
}
