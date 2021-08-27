//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Definition.h"



void setup_Lora_Send() { 
  

  Serial.println("LoRa Send Test");
  
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
 
}

void Lora_send(String Chaine) {



   LoRa.beginPacket();
   LoRa.print(Chaine);
   LoRa.endPacket();

   delay(10);
  
}

int Lora_read()
{
   int packetSize = LoRa.parsePacket();
   if (packetSize) {
    //received a packet
    Serial.print("Received packet ");

    while (LoRa.available()) {
      LoRaData = LoRa.readString();
      Serial.print(LoRaData);
    }

    
    int rssi = LoRa.packetRssi();
    Serial.print(" with RSSI ");    
    Serial.println(rssi);
    return 1;
    
    }
    else
      return 0;

    //print RSSI of packet
    
}
