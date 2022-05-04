#include <LoRa.h> 

struct LoRa_param_s {
  int  txPower = 15; // 2 to 17 dB limited to 15 because the UCA antenna = +2dB
  long frequency_id = 2; // 863.2 to 868.1 Mhz
  long frequency_offset = 0;
  int  spreadingFactor = 12; // 7 to 12
  long signalBandwidth_id = 1; // 125, 250, 500 kHz tester 62.5
  int  codingRateDenominator = 8 ; // 5 to 8
  long preambleLength = 12; // 6 to 65535, 12 default
  int  syncWord = 0x12F2; // 1 to 8 bytes ; Note Sync Word values containing 0x00 byte(s) are forbidden
  //uint8_t OCP; // Over Current Protection control (mA)
} LoRa_param_1;

long frequency_values[] = {863.2E6, 863.5E6, 863.8E6, 864.1E6, 864.4E6, 864.7E6, 865.2E6, 865.5E6,
                           865.8E6, 866.1E6, 866.4E6, 867.7E6, 867.0E6, 868.0E6, 868.1E6}; // Hz
                           
long signalBandwidth_values[] = {62500, 125000, 250000, 500000}; // Hz

//Initialisation du Lora avec les données de connexion de la structure
//Attention entre la gateway et la carte il faut que les données entrées soit identiques
void LoRa_start(struct LoRa_param_s  LR) {
  if (!LoRa.begin(frequency_values[LR.frequency_id])) {
    Serial.println("LoRa init failed. Check your connections.");
    //delay(3000);
    /*#if( MODULE == 'G')
    while(1);
    #else
    go_to_sleep (sleep_time_values[sleep_time_night_id], OLED_OFF);
    #endif*/
  }
  
  Serial.println("LoRa Frequency: ");
  Serial.println(frequency_values[LR.frequency_id]);
    
  Serial.println("LoRa Spreading Factor: "); 
  Serial.println(LR.spreadingFactor);
  LoRa.setSpreadingFactor(LR.spreadingFactor);

  Serial.println("LoRa Signal Bandwidth: "); 
  Serial.println(signalBandwidth_values[LR.signalBandwidth_id]);
  LoRa.setSignalBandwidth(signalBandwidth_values[LR.signalBandwidth_id]);
  
  Serial.println("LoRa codingRateDenominator: "); 
  Serial.println(LR.codingRateDenominator);
  LoRa.setCodingRate4(LR.codingRateDenominator);
  
  Serial.println("LoRa preambleLength: ");
  Serial.println(LR.preambleLength);
  LoRa.setPreambleLength(LR.preambleLength); 
  
  Serial.println("LoRa txPower: ");
  Serial.println(LR.txPower);
  LoRa.setTxPower(LR.txPower);
  
  Serial.println("LoRa enable CRC \n");
  
  LoRa.enableCrc(); 
}

//Permet d'envoyer la trame octets par octets
//La fonction prend en parametre: la trame, la taille de la trame et un bit d'erreur
//*** payload LoRa ********************************************************
void LoRa_send_payload(uint8_t *payload, uint8_t payload_len, uint8_t check_byte) {
  Serial.println("LoRa send:");
  LoRa.beginPacket();
  
  for (uint8_t i = 0; i <= payload_len; i++ ) {  
    LoRa.write(payload[i]);
    Serial.println(payload[i]);
    Serial.println(".");
  }

  LoRa.write(check_byte);
  Serial.println(check_byte);
 
  
  LoRa.endPacket();
}

//Stop le Lora
//*** stop LoRa ********************************************************
void LoRa_stop(){
  LoRa.sleep();
  LoRa.end();
}

//Routine d'envoie
void lora_send_trame(uint8_t *payload){
  LoRa_start(LoRa_param_1); //Permet d'initialiser Lora avec les bonnes valeurs
  LoRa_send_payload(payload,PAYLOAD_LEN,0); //Permet d'envoyer le payload
  LoRa_stop(); //Stop Lora
}
