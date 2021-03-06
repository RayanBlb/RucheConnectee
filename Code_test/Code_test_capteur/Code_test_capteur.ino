#include "Adafruit_CCS811.h" //librerie du capteur CO2
#include "arduinoFFT.h"

// Variable defini
#define MEASURE 13

// Variable Son
#define CHANNEL_SON 4

// Variable Piezo
#define CHANNEL_PIEZO 2

// Taille payload
#define PAYLOAD_LEN 20

// Structure données
struct data_trame
{
  uint8_t mac[6];
  double son;
  double piezo;
  uint16_t CO2;
  uint16_t CO2_TVOC;
  float Temperature;
} ds;

// Variable son
const uint16_t samples = 1024;         // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 2000; // Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

// Variable piezo
const uint16_t samples_piezo = 1024;         // This value MUST ALWAYS be a power of 2
const double samplingFrequency_piezo = 2000; // Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us_piezo;
unsigned long microseconds_piezo;

double vReal_piezo[samples_piezo];
double vImag_piezo[samples_piezo];

Adafruit_CCS811 ccs; // permet d'utiliser le capteur CO2 et Temperature

arduinoFFT FFTP = arduinoFFT();
arduinoFFT FFT = arduinoFFT();

void setup()
{

  Serial.begin(9600);
  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE, HIGH);

  if (!ccs.begin(0x5B))
  {
    Serial.println("Failed to start sensor! Please check your wiring.");
    while (1);
  }

  // Calibrage capteur température
  while (!ccs.available());
  ds.Temperature = ccs.calculateTemperature();
  ccs.setTempOffset(ds.Temperature - 25.0);

  // Calibrage capteur son
  sampling_period_us_piezo = round(1000000 * (1.0 / samplingFrequency_piezo));
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
}

void loop()
{
  read_CO2();
  read_CO2_TVOC();
  read_Temperature();
  //read_Piezo();
  read_Son();
  read_mac(ds.mac);
  
  //Serial.println(ds.piezo);
  Serial.println("//");
  Serial.println(ds.son);
  Serial.println("//");
  Serial.println(ds.Temperature);
  Serial.println("//");
  Serial.println(ds.CO2);
  Serial.println("//");
  Serial.println(ds.CO2_TVOC);
  Serial.println("//");
  Serial.printf("mac : %02X:%02X:%02X:%02X:%02X:%02X \n", ds.mac[0], ds.mac[1], ds.mac[2], ds.mac[3], ds.mac[4], ds.mac[5]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_Temperature()
{
  char buff[10];

  if (!ccs.readData())
  {
    ds.Temperature = ccs.calculateTemperature();
  }
  else
  {
    Serial.println("Sensor read ERROR!");
    ccs.readData();
  }

  ds.Temperature += 20; // Permet de ne pas avoir de température négative

  sprintf(buff, "%.1f", ds.Temperature); // Permet d'avoir une valeur un chiffre après la virgule
  ds.Temperature = strtof(buff, NULL);   // Char to int
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_CO2()
{
  if (!ccs.readData())
  {
    ds.CO2 = ccs.geteCO2();
  }
  else
  {
    Serial.println("Sensor read ERROR!");
    ccs.readData();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_CO2_TVOC()
{
  if (!ccs.readData())
  {
    ds.CO2_TVOC = ccs.getTVOC();
  }
  else
  {
    Serial.println("Sensor read ERROR!");
    ccs.readData();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_mac(uint8_t *mac)
{
  esp_efuse_read_mac(mac);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_Piezo()
{
  microseconds = micros();
  for (int i = 0; i < samples_piezo; i++)
  {
    vReal_piezo[i] = analogRead(CHANNEL_PIEZO);
    vImag_piezo[i] = 0;
    while (micros() - microseconds < sampling_period_us_piezo)
    {
      // empty loop
    }
    microseconds += sampling_period_us;
  }

  FFTP.Windowing(vReal_piezo, samples_piezo, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFTP.Compute(vReal_piezo, vImag_piezo, samples_piezo, FFT_FORWARD);
  FFTP.ComplexToMagnitude(vReal_piezo, vImag_piezo, samples_piezo);
  ds.piezo = FFTP.MajorPeak(vReal_piezo, samples_piezo, samplingFrequency_piezo);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_Son()
{
  microseconds = micros();
  for (int i = 0; i < samples; i++)
  {
    vReal[i] = analogRead(CHANNEL_SON);
    vImag[i] = 0;
    while (micros() - microseconds < sampling_period_us)
    {
      // empty loop
    }
    microseconds += sampling_period_us;
  }

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  ds.son = FFT.MajorPeak(vReal, samples, samplingFrequency);
}
