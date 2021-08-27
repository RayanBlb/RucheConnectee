#include "arduinoFFT.h"
#include "ExtVarGlobales.h"
#include "Definition.h"

arduinoFFT FFTP = arduinoFFT(); /* Create FFT object */

void setup_piezo()
{
  
  sampling_period_us_piezo = round(1000000*(1.0/samplingFrequency_piezo));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}

double read_piezo()
{
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
