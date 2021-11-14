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

//Variable son
const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2 
const double samplingFrequency = 2000; //Hz, must be less than 10000 due to ADC 

unsigned int sampling_period_us;
unsigned long microseconds;
       
double vReal[samples];
double vImag[samples];

//Variable piezo
const uint16_t samples_piezo = 1024; //This value MUST ALWAYS be a power of 2 
const double samplingFrequency_piezo = 2000; //Hz, must be less than 10000 due to ADC 

unsigned int sampling_period_us_piezo;
unsigned long microseconds_piezo;
       
double vReal_piezo[samples_piezo];
double vImag_piezo[samples_piezo];

Adafruit_CCS811 ccs; //permet d'utiliser le capteur CO2

arduinoFFT FFTP = arduinoFFT();
arduinoFFT FFT = arduinoFFT();

void setup_donnees() {

	if(!ccs.begin(0x5B)){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  //Calibrage capteur température
  while(!ccs.available());
  temperature = ccs.calculateTemperature();
  ccs.setTempOffset(temperature - 25.0);

  sampling_period_us_piezo = round(1000000*(1.0/samplingFrequency_piezo));
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}

float read_Temperature(){
  if(ccs.available()){
    temperature = ccs.calculateTemperature();
  }else{
    Serial.println("Sensor read ERROR!");
    ccs.readData();
    }
  return temperature;
}


int read_CO2(){
  int CO2_info;
	if(!ccs.readData()){
    CO2_info = ccs.geteCO2();
	}else{
		Serial.println("Sensor read ERROR!");
		ccs.readData();
	}
 return CO2_info;
}

int read_CO2_TVOC(){
  int CO2_info;
  if(!ccs.readData()){
    CO2_info = ccs.getTVOC();
  }else{
    Serial.println("Sensor read ERROR!");
    ccs.readData();
  }
 return CO2_info;
}

uint8_t read_mac(uint8_t *mac){
  esp_efuse_read_mac(mac);
  //Serial.printf("mac : %02X:%02X:%02X:%02X:%02X:%02X \n",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return *mac;
}

double read_Piezo(){
  /*SAMPLING*/
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

double read_Son(){
  /*SAMPLING*/
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

uint8_t build_trame(uint8_t *payload){
  
  ds.type = 'I';
  read_mac(ds.mac);
  ds.son = read_Son();
  ds.temperature = read_Temperature();
  ds.CO2 = read_CO2();
  ds.CO2_TVOC = read_CO2_TVOC();
  ds.Piezo = read_Piezo();
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
