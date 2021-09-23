/*
This software has used: https://github.com/AwotG/Arduino-MAX17055_Driver/blob/master/Arduino-MAX17055_Driver.cpp

/**********************************************************************
*
* MIT License
*
* Copyright (c) 2018 Awot Ghirmai
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
* 
* Authors: 
* Awot Ghirmai; ghirmai.awot@gmail.com
* Ole Dreessen; ole.dreessen@maximintegrated.com
* 
**********************************************************************

15-11-2019
Author:
Albert van Dalen 
https://www.avdweb.nl/arduino/hardware-interfacing/max17263-fuel-gauge
*/

void setMax17263memberVariables(long designCap_mAh, bool r100, bool vChg, uint8_t modelID) 
{ max17263.designCap_mAh = designCap_mAh/capacity_multiplier_mAH; 
  max17263.r100 = r100; 
  max17263.vChg = vChg; 
  max17263.modelID = modelID; 
}

void printMultipliers() 
{ Serial << F("\ncapacity_multiplier_mAH ") << _FLOAT(capacity_multiplier_mAH, 8);
  Serial << F("\ncurrent_multiplier_mV ") << _FLOAT(current_multiplier_mV, 8);
}

void printFuelGaugeResults() 
{ 
  Serial << F("\n\nFuelGaugeResults:");
  Serial << F("\nSOC ") << _FLOAT(max17263.getSOC(), 1) << F(" %");
  Serial << F(" Vcell ") << _FLOAT(max17263.getVcell(), 2) << F(" V");
  Serial << F("\nCap ") << _FLOAT(max17263.getCapacity_mAh()*capacity_multiplier_mAH, 2) << F(" mAH");
  Serial << F("\nCurrent ") << _FLOAT(max17263.getCurrent()*current_multiplier_mV, 2) << F(" A"); 
  Serial << F("\nPower ") << _FLOAT(max17263.getPower(), 2) << F(" W");
  Serial << F("\nTTE ") << _FLOAT(max17263.getTimeToEmpty(), 2) << F(" h");
  Serial << F("\nTemp ") << _FLOAT(max17263.getTemp(), 1) << F(" C");
  Serial << endl;
}

void printHistory()
{ Serial << F("\nreadHistory");
  Serial << F("\nRCOMP0 ") << history.RCOMP0;
  Serial << F("\nTempCo ") << history.TempCo;
  Serial << F("\nFullCapRep ") << history.FullCapRep;
  Serial << F("\nCycles ") << history.Cycles;
  Serial << F("\nFullCapNom ") << history.FullCapNom;
  Serial << endl;
}

void writeHistoryToEEPROM() // todo
{  
}

void readHistoryFromEEPROM() // todo
{ 
}

void manageMAX17263init()
{ if(max17263.powerOnResetEvent()) 
  { max17263.init(); // Step 0 and 3.2 periodically check reset and than initialize (MAX17263 is erased after power down)
    //readHistoryFromEEPROM();// todo
    //max17263.restoreHistory(history); // todo, because MAX17263 was erased after power down
  }
}

void manageMAX17263historyRead()
{ static bool stopHistoryReading = 0;
  if((millis() > readHistoryAfter_minutes*60000) && !stopHistoryReading) // read history after some time after start driving
  { max17263.readHistory(history); // Step 3.5 save Learned Parameters to EEPROM (not done: every 64% change in the battery)  
    stopHistoryReading = 1;
    writeHistoryToEEPROM(); // todo
    //printHistory(); // todo
  }
}
