#include "Adafruit_CCS811.h" //librerie du capteur CO2
#include "arduinoFFT.h" 
#include "Definition.h"
#include "WiFi.h"

const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2 
const double samplingFrequency = 2000; //Hz, must be less than 10000 due to ADC 

unsigned int sampling_period_us;
unsigned long microseconds;
       
double vReal[samples];
double vImag[samples];

const uint16_t samples_piezo = 1024; //This value MUST ALWAYS be a power of 2 
const double samplingFrequency_piezo = 2000; //Hz, must be less than 10000 due to ADC 

unsigned int sampling_period_us_piezo;
unsigned long microseconds_piezo;
       
double vReal_piezo[samples_piezo];
double vImag_piezo[samples_piezo];

Adafruit_CCS811 ccs; //permet d'utiliser le capteur CO2

arduinoFFT FFTP = arduinoFFT();
arduinoFFT FFT = arduinoFFT();

float temperature;

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


String read_CO2(){
  String CO2_info;
	if(!ccs.readData()){
    CO2_info = String(ccs.geteCO2())+"|"+String(ccs.getTVOC());
	}else{
		Serial.println("Sensor read ERROR!");
		ccs.readData();
	}
 return CO2_info;
}

String read_mac(){
 return String(WiFi.macAddress());
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
