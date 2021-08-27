#include "arduinoFFT.h"
#include "ExtVarGlobales.h"
#include "Definition.h"



arduinoFFT FFT = arduinoFFT(); /* Create FFT object */




void setup_son()
{
  
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}

double read_son()
{
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
