#include "Adafruit_CCS811.h" //librerie du capteur CO2
#include "arduinoFFT.h"
#include "WiFi.h"
#include "LoRa.h"

Adafruit_CCS811 ccs; //permet d'utiliser le capteur CO2

float cap_temp;
uint16_t cap_co2;
uint16_t cap_tvoc;

struct data_trame { 
  char type; 
  uint8_t mac[6];
  double son;
  float temperature;
  uint16_t CO2;
  uint16_t CO2_TVOC;
  double Piezo;
  uint16_t hygro;
  uint16_t v_bat;
  uint16_t charg_bat;
  uint16_t IA;
  int erreur;
} ds;

/**************************************/
//  Variable Lora                     //
/**************************************/

//define the pins used by the LoRa transceiver module
#define SCK 18
#define MISO 19
#define MOSI 23
#define CS 18

#define SS 5
#define RST 14
#define DI0 26

/**************************************/
//  Variable Fonction                 //
/**************************************/

//Viable defini
#define MEASURE 13

//  Variable Son 
#define CHANNEL_SON 4

//  Variable Piezo 
#define CHANNEL_PIEZO 2

//Taille payload
#define PAYLOAD_LEN 25


void setup() {
  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,HIGH);

  if(!ccs.begin(0x5B)){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  //Calibrage capteur température
  while(!ccs.available());
  ds.temperature = ccs.calculateTemperature();
  ccs.setTempOffset(ds.temperature - 25.0);

  SPI.begin(SCK, MISO, MOSI, CS);
  LoRa.setPins(SS, RST, DI0);
  
  Serial.begin(9600);
}

void loop() { 
  uint16_t dummy;
  read_Temperature();
  dummy = read_CO2();
  cap_tvoc = read_CO2_TVOC();

  Serial.print("Température : ");
  Serial.print(ds.temperature);
  Serial.print("\tCO2 : ");
  Serial.println(dummy);
  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_Temperature(){ //permet de récuperer la température
  char buff [10];
  
  if(ccs.available()){
    ds.temperature = ccs.calculateTemperature();
  }else{
    Serial.println("Sensor read ERROR!");
    ccs.readData();
  }
  
  sprintf(buff,"%.1f",ds.temperature); //Permet d'avoir une valeur un chiffre après la virgule
  ds.temperature = strtof(buff, NULL); //Char to int
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t read_CO2(){ //Permet de récupere la mesure de dioxyde de carbone estimée stockée
  //uint16_t CO2_info;
  if(!ccs.readData()){
    //CO2_info = ccs.geteCO2();
    ds.CO2 = ccs.geteCO2();
  }else{
    Serial.println("Sensor read ERROR!");
    ccs.readData();
  }
 //return CO2_info;
 return ds.CO2;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t read_CO2_TVOC(){ //permet de récupere la mesure des composés organiques volatils totaux stockés
  uint16_t CO2_info;
  if(!ccs.readData()){
    CO2_info = ccs.getTVOC();
  }else{
    Serial.println("Sensor read ERROR!");
    ccs.readData();
  }
 return CO2_info;
}
/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t read_mac(uint8_t *mac){//Permet de récuperer l'adresse mac
  esp_efuse_read_mac(mac);
  //Serial.printf("mac : %02X:%02X:%02X:%02X:%02X:%02X \n",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
double read_Piezo(){
 microseconds = micros();
  for(int i=0; i<samples_piezo; i++)
  {
      vReal_piezo[i] = analogRead(CHANNEL_PIEZO);
      vImag_piezo[i] = 0;
      while(micros() - microseconds < sampling_period_us_piezo){
        //empty loop
      }
      microseconds += sampling_period_us;
  }

  FFTP.Windowing(vReal_piezo, samples_piezo, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
  FFTP.Compute(vReal_piezo, vImag_piezo, samples_piezo, FFT_FORWARD); 
  FFTP.ComplexToMagnitude(vReal_piezo, vImag_piezo, samples_piezo); 
  double x = FFTP.MajorPeak(vReal_piezo, samples_piezo, samplingFrequency_piezo);
  
  return x;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
double read_Son(){
 microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL_SON);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); 
  FFT.ComplexToMagnitude(vReal, vImag, samples); 
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  
  return x;
}
*/
