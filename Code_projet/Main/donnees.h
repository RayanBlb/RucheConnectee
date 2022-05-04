#include <LITTLEFS.h>
#include <driver/i2s.h>
#include "Adafruit_CCS811.h" //librerie du capteur CO2 et temperature
#include "arduinoFFT.h"
#include "WiFi.h"

#include "ia.h"
#include "audio.h"
#include "config.h"

// Structure données
struct data_trame
{
  char type;
  uint8_t mac[6];
  double son;
  double piezo;
  uint16_t CO2;
  uint16_t CO2_TVOC;
  float Temperature;
  uint8_t hygro;
  uint16_t v_bat;
  uint8_t charg_bat;
  uint8_t IA;
  int erreur;
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

Adafruit_CCS811 ccs; // permet d'utiliser le capteur CO2 et temperature

arduinoFFT FFTP = arduinoFFT();
arduinoFFT FFT = arduinoFFT();

//Fonction de setup
void setup_donnees()
{

  if (!ccs.begin(0x5B))
  {
    Serial.println("Failed to start sensor! Please check your wiring.");
    while (1)
      ;
  }

  // Calibrage capteur température
  while (!ccs.available())
    ;
  ds.Temperature = ccs.calculateTemperature();
  ccs.setTempOffset(ds.Temperature - 25.0);

  // Calibrage capteur son
  sampling_period_us_piezo = round(1000000 * (1.0 / samplingFrequency_piezo));
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  // Setup IA
  LITTLEFSInit();
}

void read_Temperature()
{ // permet de récuperer la température
  char buff[10];

  if (ccs.available())
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

void read_CO2()
{ // Permet de récupere la mesure de dioxyde de carbone estimée stockée

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

void read_CO2_TVOC()
{ // permet de récupere la mesure des composés organiques volatils totaux stockés

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

void read_mac(uint8_t *mac)
{ // Permet de récuperer l'adresse mac
  esp_efuse_read_mac(mac);
  // Serial.printf("mac : %02X:%02X:%02X:%02X:%02X:%02X \n",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void read_Piezo()
{
  /*SAMPLING*/
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

void read_Son()
{
  /*SAMPLING*/
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

//Fonction IA, permet de récuperer l'état de la ruche on fonction de 30 seconde d'audio
int info_ia()
{
  int resultat = 0;

  recording_file_init();
  i2sInit();
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, NULL);
  delay(RECORD_TIME * 1000 + 2500);
  resultat = ai();

  return resultat;
}

void build_trame(uint8_t *payload)
{ // permet de fabriquer la trame

  //Récuperation des données afin de remplir la structure
  ds.type = 'I';
  read_mac(ds.mac);
  read_Son();
  read_Temperature();
  read_CO2();
  read_CO2_TVOC();
  read_Piezo();

  ds.hygro = 30;
  ds.v_bat = 13;
  ds.charg_bat = 255;
  ds.IA = info_ia();

  ds.erreur = 0;

  Serial.printf("Trame before cast : %c | %X:%X:%X:%X:%X:%X | %f | %f | %d | %d | %f | %d | %d | %d | %d | %d\n", ds.type, ds.mac[0], ds.mac[1], ds.mac[2], ds.mac[3], ds.mac[4], ds.mac[5], ds.son, ds.Temperature, ds.CO2, ds.CO2_TVOC, ds.piezo, ds.hygro, ds.v_bat, ds.charg_bat, ds.IA, ds.erreur);
   //Conversion en uint8
  payload[0] = uint8_t(ds.type);

  for (int i = 1; i < 7; i++)
  {
    payload[i] = uint8_t(ds.mac[i - 1]);
  }

  //Split d'un uin16 en 2 uint8
  payload[7] = uint8_t((uint16_t(ds.son) & 0xFF00) >> 8);
  payload[8] = uint8_t((uint16_t(ds.son) & 0x00FF));

  payload[9] = uint8_t((uint16_t(ds.Temperature * 10) & 0xFF00) >> 8); // Multiplie la valeur de la température par 10 pour ne pas avoir de nombre a virgule
  payload[10] = uint8_t((uint16_t(ds.Temperature * 10) & 0x00FF));

  payload[11] = uint8_t((uint16_t(ds.CO2) & 0xFF00) >> 8);
  payload[12] = uint8_t((uint16_t(ds.CO2) & 0x00FF));

  payload[13] = uint8_t((uint16_t(ds.CO2_TVOC) & 0xFF00) >> 8);
  payload[14] = uint8_t((uint16_t(ds.CO2_TVOC) & 0x00FF));

  payload[15] = uint8_t((uint16_t(ds.piezo) & 0xFF00) >> 8);
  payload[16] = uint8_t((uint16_t(ds.piezo) & 0x00FF));

  payload[17] = ds.hygro;

  payload[18] = ds.v_bat;

  payload[19] = ds.charg_bat;

  payload[20] = ds.IA;
}
