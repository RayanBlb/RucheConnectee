#include "son.h"
#include "CO2.h"
#include "Lora_Send.h"
#include "piezo.h"
#include "VarGlobales.h"
#include "Definition.h"

void setup() {

  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,HIGH);
  
  Serial.begin(115200);

  setup_CO2();
  setup_son();
  setup_Lora_Send();
  setup_piezo();
  Serial.begin(9600);
}

void loop() {
 
 //Serial.println(read_son()); 
 //Serial.println(read_CO2_value());
 
 //Programmation de la trame générale
 SendLoraChaine = "Freq: " + String(read_son()) +  "FreqP: " + String(read_piezo()) +  "Hz, " + read_CO2_value();
 Serial.println(SendLoraChaine);
 //Envoie des data via lora
 /*Lora_send(SendLoraChaine);
 Counter = 0;
 while(Lora_read() < 1 && Counter <4)
 {
    delay(60);
    Seiral.println("Send data again");
    Lora_send(SendLoraChaine);//envoie du packet
    Counter++;    
 }
 Serial.println("Going to sleep");
 esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
 esp_deep_sleep_start();
 
 delay(10);*/
}
