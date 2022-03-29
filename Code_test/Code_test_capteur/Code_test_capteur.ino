#include "Adafruit_CCS811.h" //librerie du capteur CO2
#include "arduinoFFT.h"

//Structure données
struct data_trame { 
  char type;
  uint8_t mac[6];
  double son;
  double Piezo;
  uint16_t CO2;
  uint16_t CO2_TVOC;
  float temperature;
  uint8_t hygro;
  uint16_t v_bat;
  uint8_t charg_bat;
  uint8_t IA;
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

Adafruit_CCS811 ccs; //permet d'utiliser le capteur CO2 et temperature

arduinoFFT FFTP = arduinoFFT();
arduinoFFT FFT = arduinoFFT();

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
#define PAYLOAD_LEN 20

void setup() {
  Serial.begin(9600);
  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,HIGH);

  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  //Calibrage capteur température
  while(!ccs.available());
  ds.temperature = ccs.calculateTemperature();
  ccs.setTempOffset(ds.temperature - 25.0);
}

void loop() { 
  read_Temperature_CO2_TVOC();
  read_mac(ds.mac);
  //Serial.println(ds.mac);
  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_Temperature_CO2_TVOC(){
  char buff [10];
  
  if(ccs.available()){
    ds.temperature = ccs.calculateTemperature();
    if(!ccs.readData()){
      ds.CO2 = ccs.geteCO2();
      ds.CO2_TVOC = ccs.getTVOC();

      Serial.println("CO2: ");
      Serial.println(ds.CO2);
      Serial.println("ppm, TVOC: ");
      Serial.println(ds.CO2_TVOC);
      Serial.println("ppb Temp:");
      Serial.println(ds.temperature);
      }else{
        Serial.println("ERROR!");
        while(1);
        }
        }
  sprintf(buff,"%.1f",ds.temperature); //Permet d'avoir une valeur un chiffre après la virgule
  ds.temperature = strtof(buff, NULL); //Char to int
  Serial.println("ppb Temp:");
  Serial.println(ds.temperature);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t read_mac(uint8_t *mac){//Permet de récuperer l'adresse mac
  esp_efuse_read_mac(mac);
  //Serial.printf("mac : %02X:%02X:%02X:%02X:%02X:%02X \n",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
/*
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
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_Son(){
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
  //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  ds.son = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //return x;
}
//*/
