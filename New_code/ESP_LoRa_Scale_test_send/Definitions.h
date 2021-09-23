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

#ifndef Max17263Definitions_h
#define Max17263Definitions_h

const unsigned long dataRepeadPeriod_ms = 250;
const unsigned long readHistoryAfter_minutes = 10; // todo 

/* Battery parameters Maniax Power Panasonic 90Ah PANASONIC CGR18650CG Li-Ion 3.6V */
//float rSense = 0.002;  
float rSense = 0.010;
//long designCap_mAh = 90000; // max 163000mAh 
//long designCap_mAh = 7000;
//bool r100 = 0; // if NTC > 100k
//bool vChg = 1; // default=1, if charge voltage > 4.25V (4.3V–4.4V), influence on SOC 0:58.8% / 1:50.4% 
//bool vChg = 0;
// 0: for most lithium cobalt-oxide variants. Supported by EZ without characterization
// 2: for lithium NCR or NCA cells such as Panasonic. Supported by EZ without characterization
// 6: for LiFePO4, custom characterization is recommended, instead of an EZ configuration
//uint8_t modelID = 0; // not LiFePO4
//uint8_t modelID = 6; // LiFePO4
const float capacity_multiplier_mAH = (5e-6)/rSense; // UG6595 page 4, 0.0025 at 2mOhm, 
const float current_multiplier_mV = (1.5625e-6)/rSense; // UG6595 page 4, 0.00078125 at 2mOhm
#endif
