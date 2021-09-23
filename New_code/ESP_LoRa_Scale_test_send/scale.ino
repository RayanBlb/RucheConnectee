#include "global_functions.h"

//*** **********************************************************
void initScale(uint8_t sensor_nb, uint8_t action) {
  bool result = true; //Accumulate a result as we do the setup

  // scale 0 begin without reset 
  if (scale0.begin(Wire, false) == false) { // TESTER OLED sur ACTION 
    DEBUGPRINTLN2("Scale 0 not detected. Please check wiring.");
    go_to_sleep (sleep_time_values[sleep_time_night_id], action); // action = 0 => OLED OFF
  }
  DEBUGPRINTLN2("Scale 0 detected!");
  // reset & configuration (no calibration)
  result &= scale0.reset();//Reset all registers
  result &= scale0.powerUp(); //Power on analog and digital sections of the scale
  result &= scale0.setLDO(ldoValue);//Set LDO to 3.3V
  result &= scale0.setGain(gainValue);
  result &= scale0.setSampleRate(rate); //Set samples per second to 10
  result &= scale0.setRegister(NAU7802_ADC, 0x30); //Turn off CLK_CHP. From 9.1 power on sequencing.
  //result &= scale0.setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
  if (!result) {
      DEBUGPRINTLN2("Scale 0 : setup aborted.");
      go_to_sleep (sleep_time_values[sleep_time_night_id], action); // action = 0 => OLED OFF
  }
  DEBUGPRINTLN2("Scale 0 configurated!");

  // scale 1 begin without reset 
  if ( sensor_nb == 4 ) {
    result = true;
    // begin without reset 
    if (scale1.begin(Wire1, false) == false) { // TESTER OLED sur ACTION 
      DEBUGPRINTLN2("Scale 1 not detected. Please check wiring.");
      go_to_sleep (sleep_time_values[sleep_time_night_id], action); // action = 0 => OLED OFF
    }
    DEBUGPRINTLN2("Scale 1 detected!");
    // reset & configuration (no calibration)
    result &= scale1.reset();//Reset all registers
    result &= scale1.powerUp(); //Power on analog and digital sections of the scale
    result &= scale1.setLDO(ldoValue);//Set LDO to 3.3V
    result &= scale1.setGain(gainValue);
    result &= scale1.setSampleRate(rate); //Set samples per second to 10
    result &= scale1.setRegister(NAU7802_ADC, 0x30); //Turn off CLK_CHP. From 9.1 power on sequencing.
    //result &= scale1.setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
    if (!result) {
        DEBUGPRINTLN2("Scale 1 : setup aborted.");
        go_to_sleep (sleep_time_values[sleep_time_night_id], action); // action = 0 => OLED OFF
    }
    DEBUGPRINTLN2("Scale 1 configurated!");
    //scale1.powerDown();
  }
}

//*** **********************************************************
void calibrateScale(uint8_t sensor_nb) {
  int8_t  digit[]={-13,-13,-13,-13,-13}; // -13+'0'='#'
  int8_t  d = 0, i = 0;
  char    buf[15];
  uint8_t next = 0;
  uint8_t next_mem = 0;
  uint8_t valid = 0;
  uint8_t valid_mem = 0;
  float   temperature=0.0;
  
  DEBUGPRINTLN2("Scale calibration");

  // zero offset -------------
  if ( sensor_nb == 1 )
    oled_print(OB16, CALIBRATION1, CALIBRATION2, CALIBRATION3, CALIBRATION4);
  else
    oled_print(OB16, CALIBRATION1, CALIBRATION2, CALIBRATION3A, CALIBRATION4);
  while ( !digitalRead(NEXT_BTN) );
  // sensor A ***
  if ( sensor_nb == 1 )
    oled_print(OB16, ZERO1, ZERO2, ZERO3, ZERO4);
  else
    oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"A", ZERO4);
  DEBUGPRINTLN2("------- offset A ---");
  while ( get_1sigma_Average(SENSOR_0, &zeroOffset[0], CAL_SAMPLES, CAL_READINGS) ) {  // get zeroOffset
    Serial.println("error offset A");
    oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"A", "ERROR");
  }
  DEBUGPRINT3("zeroOffset[0]="); DEBUGPRINTLN3(zeroOffset[0]);
  
  if ( sensor_nb > 1 ) {
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    // sensor B ***
    oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"B", ZERO4);
    DEBUGPRINTLN2("------- offset B ---");
    while ( get_1sigma_Average(SENSOR_1, &zeroOffset[1], CAL_SAMPLES, CAL_READINGS) ) {  // get zeroOffset
      Serial.println("error offset B");
      oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"B", "ERROR");
    }
  DEBUGPRINT3("zeroOffset[1]="); DEBUGPRINTLN3(zeroOffset[1]);
  }

  if ( sensor_nb == 4 ) {
    scale1.powerUp(); delay(100);
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    // sensor C ***
    oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"C", ZERO4);
    DEBUGPRINTLN2("------- offset C ---");
    while ( get_1sigma_Average(SENSOR_2, &zeroOffset[2], CAL_SAMPLES, CAL_READINGS) ) {  // get zeroOffset
      Serial.println("error offset C");
      oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"C", "ERROR");
    }
    DEBUGPRINT3("zeroOffset[2]="); DEBUGPRINTLN3(zeroOffset[2]);
    // sensor D ***
    oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"D", ZERO4);
    DEBUGPRINTLN2("------- offset D ---");
    while ( get_1sigma_Average(SENSOR_3, &zeroOffset[3], CAL_SAMPLES, CAL_READINGS) ) {  // get zeroOffset
      Serial.println("error offset D");
      oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"D", "ERROR");
    }
    DEBUGPRINT3("zeroOffset[3]="); DEBUGPRINTLN3(zeroOffset[3]); 
  }

  // define the known weight -------------
  do {
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    //if ( know_weight == 0 ) {
      digit[0]=-13; digit[1]=-13; digit[2]=-13; digit[3]=-13; 
    //}
    d = 4; 
    digit[d] = 0;
    sprintf(buf,"%c%c.%c%c%c kg",digit[4]+'0',digit[3]+'0',digit[2]+'0',digit[1]+'0',digit[0]+'0');
    oled_print(OC16B2, MASS1, buf, MASS3, MASS4);
    valid_mem = 1; next_mem = 1;
    do {
        valid = digitalRead(VALID_BTN);
        if ( valid ) {
          if ( valid_mem == 0 ) {
            valid_mem = 1;
            d--;
            digit[d] = 0;
            sprintf(buf,"%c%c.%c%c%c kg",digit[4]+'0',digit[3]+'0',digit[2]+'0',digit[1]+'0',digit[0]+'0');
            oled_print(OC16B2, MASS1, buf, MASS3, MASS4);
          }
        } else {
          valid_mem = 0;
        }    
        next = digitalRead(NEXT_BTN);
        if ( next ) {
          if ( next_mem == 0 ) {
            next_mem = 1;
            digit[d]++;
            if ( digit[d] > 9 ) 
              digit[d] = 0; 
            sprintf(buf,"%c%c.%c%c%c kg",digit[4]+'0',digit[3]+'0',digit[2]+'0',digit[1]+'0',digit[0]+'0');
            oled_print(OC16B2, MASS1, buf, MASS3, MASS4);
          }
        } else {
          next_mem = 0;
        }
    } while ( d >= 0 );
    oled_print(OC16B2, MASS1, buf, "", "");
    know_weight = digit[4]*10000 + digit[3]*1000 + digit[2]*100 + digit[1]*10 + digit[0]; // grams
    DEBUGPRINT3("know_weight:"); DEBUGPRINTLN3(know_weight);
    delay(AFF_TIME);
  } while ( know_weight == 0 );

  // sensors calibration -------------
  timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
  // sensor A ***
  scale0.setChannel(NAU7802_CHANNEL_1); // Select channel before Reading
  while ( !digitalRead(NEXT_BTN) ) {
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    oled_print(OB16, CAL1, CAL2 + (String)" A", String(scale0.getAverage(5)), CAL4);
    delay(200);
  }
  while ( digitalRead(NEXT_BTN) );
  oled_print(OB16, CAL5, CAL6, CAL7 + (String)" A", CAL8);
  DEBUGPRINTLN2("------- calibration A ---");
  while ( get_1sigma_Average(SENSOR_0, &onScale, CAL_SAMPLES, CAL_READINGS) ) {  // get calibration factor
    Serial.println("error calibration A");
    oled_print(OB16, CAL5, CAL6, CAL7 + (String)" A", "ERROR");
  }
  DEBUGPRINT3("onScale="); DEBUGPRINTLN3(onScale);
  calibrationFactor[0] = ((float)onScale - (float)zeroOffset[0]) / (float)(know_weight);
  DEBUGPRINT3("calibrationFactor:"); DEBUGPRINTLN3(calibrationFactor[0]);


  if ( sensor_nb > 1 ) {
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    // sensor B ***
    scale0.setChannel(NAU7802_CHANNEL_2); // Select channel before Reading
    while ( !digitalRead(NEXT_BTN) ) {
      timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
      oled_print(OB16, CAL1, CAL2 + (String)" B", String(scale0.getAverage(5)), CAL4);
      delay(200);
    }
    while ( digitalRead(NEXT_BTN) );
    oled_print(OB16, CAL5, CAL6, CAL7 + (String)" B", CAL8);
    DEBUGPRINTLN2("------- calibration B ---");
    while ( get_1sigma_Average(SENSOR_1, &onScale, CAL_SAMPLES, CAL_READINGS) ) {  // get calibration factor
      Serial.println("error calibration B");
      oled_print(OB16, CAL5, CAL6, CAL7 + (String)" B", "ERROR");
    }
    DEBUGPRINT3("onScale="); DEBUGPRINTLN3(onScale);
    calibrationFactor[1] = ((float)onScale - (float)zeroOffset[1]) / (float)know_weight;
    DEBUGPRINT3("calibrationFactor:"); DEBUGPRINTLN3(calibrationFactor[1]);    
  }
  
  if ( sensor_nb == 4 ) {
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    // sensor C ***
    scale1.setChannel(NAU7802_CHANNEL_1); // Select channel before Reading
    while ( !digitalRead(NEXT_BTN) ) {
      timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
      oled_print(OB16, CAL1, CAL2 + (String)" C", String(scale1.getAverage(5)), CAL4);
      delay(200);
    }
    while ( digitalRead(NEXT_BTN) );
    oled_print(OB16, CAL5, CAL6, CAL7 + (String)" C", CAL8);
    DEBUGPRINTLN3("------- calibration C ---");
    while ( get_1sigma_Average(SENSOR_2, &onScale, CAL_SAMPLES, CAL_READINGS) ) {  // get calibration factor
      Serial.println("error calibration C");
      oled_print(OB16, CAL5, CAL6, CAL7 + (String)" C", "ERROR");
    }
    DEBUGPRINT2("onScale="); DEBUGPRINTLN3(onScale);
    calibrationFactor[2] = ((float)onScale - (float)zeroOffset[2]) / (float)know_weight;
    DEBUGPRINT3("calibrationFactor:"); DEBUGPRINTLN3(calibrationFactor[2]);  
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    // sensor D ***
    scale1.setChannel(NAU7802_CHANNEL_2); // Select channel before Reading
    while ( !digitalRead(NEXT_BTN) ) {
      timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
      oled_print(OB16, CAL1, CAL2 + (String)" D", String(scale1.getAverage(5)), CAL4);
      delay(200);
    }
    while ( digitalRead(NEXT_BTN) );
    oled_print(OB16, CAL5, CAL6, CAL7 + (String)" D", CAL8);
    DEBUGPRINTLN2("------- calibration D ---");
    while ( get_1sigma_Average(SENSOR_3, &onScale, CAL_SAMPLES, CAL_READINGS) ) {  // get calibration factor
      Serial.println("error calibration D");
      oled_print(OB16, CAL5, CAL6, CAL7 + (String)" D", "ERROR");
    }
    DEBUGPRINT3("onScale="); DEBUGPRINTLN3(onScale);
    calibrationFactor[3] = ((float)onScale - (float)zeroOffset[3]) / (float)know_weight;
    DEBUGPRINT3("calibrationFactor:"); DEBUGPRINTLN3(calibrationFactor[3]);        
  }


  // global calibration, get new zero offset with loading plate------------- 
  massOffset = 0;

  if ( sensor_nb > 1 ) {
    oled_print(OB16, CALIBRATION1, CALIBRATION2, CALIBRATION3B, CALIBRATION4);
    while ( !digitalRead(NEXT_BTN) );
  
    // sensor A ***
    oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"A", ZERO4); 
    DEBUGPRINTLN2("------- new offset A ---");
    while ( get_1sigma_Average(SENSOR_0, &onScale, CAL_SAMPLES, CAL_READINGS) )   //  get new zeroOffset
      Serial.println("error sensor A");
    massOffset = ((float)onScale-(float)zeroOffset[0])/calibrationFactor[0];
    // sensor B ***
    oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"B", ZERO4); 
    DEBUGPRINTLN2("------- new offset B ---");
    while ( get_1sigma_Average(SENSOR_1, &onScale, CAL_SAMPLES, CAL_READINGS) )  //  get new zeroOffset
      Serial.println("error sensor B");
    massOffset += ((float)onScale-(float)zeroOffset[1])/calibrationFactor[1];

    if ( sensor_nb == 4 ) {
      // sensor C ***
      oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"C", ZERO4); 
      DEBUGPRINTLN2("------- new offset C ---");
      while ( get_1sigma_Average(SENSOR_2, &onScale, CAL_SAMPLES, CAL_READINGS) )  // get new zeroOffset
        Serial.println("error sensor C");
      massOffset += ((float)onScale-(float)zeroOffset[2])/calibrationFactor[2];
      // sensor D ***
      oled_print(OB16, ZERO1, ZERO2, ZERO3 + (String)"D", ZERO4); 
      DEBUGPRINTLN2("------- new offset D ---");
      while ( get_1sigma_Average(SENSOR_3, &onScale, CAL_SAMPLES, CAL_READINGS) )  // get new zeroOffset
        Serial.println("error sensor D");
      massOffset += ((float)onScale-(float)zeroOffset[3])/calibrationFactor[3];
    }
  }

  // temperature
#ifdef T_18B20
  oled_print(OB16, TEMP_MEASURE1, TEMP_MEASURE2, "", ZERO4); 
  calibration_temp = 0;
  for ( i = 0 ; i < CAL_READINGS ; i++ ) {
    //DEBUGPRINT3("=== reading : "); DEBUGPRINTLN3(i);
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    // temperature : reset the sensor and request conversion
    temp_sensor.reset();
    temp_sensor.select(temp_addr);
    temp_sensor.write(0x44, 1);        // start conversion, with parasite power on at the end
    DEBUGPRINTLN3("request temperature.");
    requestTime = millis();
    while ( millis() < requestTime + 1000 ); // 750 ms : convertion time @ resolution 12 bits
      temperature = get_temp();
      if (temperature < -50.0 || temperature > 80.0  ) {
        DEBUGPRINTLN3("temp measure error. Check wiring!");
        oled_print(OB16, TEMP_ERROR1, TEMP_ERROR2, TEMP_ERROR3, TEMP_ERROR4);
        while (1) ;
      }
      DEBUGPRINT3("temperature = "); DEBUGPRINT3(temperature); DEBUGPRINTLN3(" °C");
      oled_print(OB16, TEMP_MEASURE1, TEMP_MEASURE2, "            " + (String)temperature + " °C", ZERO4); 
      calibration_temp += temperature;
  delay(500);
  }
  temp_sensor.depower();
  calibration_temp /= CAL_READINGS;
  DEBUGPRINT3("calibration temperature = "); DEBUGPRINT3(calibration_temp); DEBUGPRINTLN3("°C");

#else
  calibration_temp = 20.0;
  temp_factor = 0.0;
#endif

  // write to eeproom
  EEPROM.put(LOCATION_CALIBRATION_TEMP,calibration_temp); 
  EEPROM.put(LOCATION_TEMP_FACTOR,temp_factor); 
  EEPROM.put(LOCATION_SCALEA_CAL_FACTOR, calibrationFactor[0]); //DEBUGPRINTLN3(calibrationFactor[0]);
  EEPROM.put(LOCATION_SCALEA_ZERO, zeroOffset[0]); //DEBUGPRINTLN3(zeroOffset[0]);
  EEPROM.put(LOCATION_SCALEB_CAL_FACTOR, calibrationFactor[1]);  //DEBUGPRINTLN3(calibrationFactor[1]);
  EEPROM.put(LOCATION_SCALEB_ZERO, zeroOffset[1]); //DEBUGPRINTLN3(zeroOffset[1]);
  EEPROM.put(LOCATION_SCALEC_CAL_FACTOR, calibrationFactor[2]);  //DEBUGPRINTLN3(calibrationFactor[2]);
  EEPROM.put(LOCATION_SCALEC_ZERO, zeroOffset[2]); //DEBUGPRINTLN3(zeroOffset[2]);
  EEPROM.put(LOCATION_SCALED_CAL_FACTOR, calibrationFactor[3]);  //DEBUGPRINTLN3(calibrationFactor[3]);
  EEPROM.put(LOCATION_SCALED_ZERO, zeroOffset[3]); //DEBUGPRINTLN3(zeroOffset[3]);
  EEPROM.put(LOCATION_KNOWN_WEIGHT, know_weight);
  EEPROM.put(LOCATION_MASS_OFFSET, massOffset);
  EEPROM.put(LOCATION_SENSOR_NB, sensor_nb); //DEBUGPRINTLN3(sensor_nb);
  EEPROM.commit();
  DEBUGPRINTLN3("Calibration write to eeprom");
  scale_calibration_flag = true;
}

//*** **********************************************************
uint8_t get_sensor_nb (uint8_t nb) {
  uint8_t next = 0;
  uint8_t next_mem = 0;
  
  if ( nb )
    oled_print(OC16, NB_OF_SENSORS1, (String)nb, CHOOSE, VALID);
  else 
    oled_print(OC16, NB_OF_SENSORS1, NB_OF_SENSORS2, CHOOSE, VALID);
  while ( !digitalRead(VALID_BTN) || nb == 0 ) {
    next = digitalRead(NEXT_BTN);
      if ( next ) {
        if ( next_mem == 0 ) {
          next_mem = 1;
          nb++;
          if ( nb > 4 ) 
            nb = 1; 
          if ( nb == 3 )
            nb++;
          oled_print(OC16, NB_OF_SENSORS1, (String)nb, CHOOSE, VALID);
        }
      } else {
        next_mem = 0;
      }
  }
  return nb;
}

//*** **********************************************************
uint8_t get_1sigma_Average (uint8_t sensor, int32_t *average_global, uint16_t samples, uint8_t readings) {
  /* 
   *  
   */
  NAU7802 *scale;
  uint8_t channel;
  uint8_t i = 0;
  uint16_t j = 0;
  uint16_t k = 0;
  double sum = 0;
  double sum_ave = 0;
  int32_t value[samples];
  int32_t average[readings];
  int32_t s[readings],sg;
  int32_t delta ;
  uint8_t sensor_id = 0;

  switch( sensor ) {
    case 0 : scale = &scale0; channel = NAU7802_CHANNEL_1; break; // sensor 0
    case 1 : scale = &scale0; channel = NAU7802_CHANNEL_2; break; // sensor 1
    case 2 : scale = &scale1; channel = NAU7802_CHANNEL_1; break; // sensor 2
    case 3 : scale = &scale1; channel = NAU7802_CHANNEL_2; break; // sensor 3
    default: go_to_sleep (sleep_time, OLED_OFF);
  }

  //Select between channel values NAU7802_CHANNEL_1 = 0, NAU7802_CHANNEL_2 = 1
  scale->setChannel(channel); //Serial.print("channel "); Serial.println(channel);

  // first measures wrong  ???  !!!
  while ( !scale->available() ) {
    DEBUGPRINT3("Scale not available - wait or check wiring - sensor : "); DEBUGPRINTLN3(sensor);
    delay(100);
  }
  /* After reset or after resuming normal operating mode after power-down mode, the host should wait
  through six cycles of data conversion. */
  for ( i = 0 ; i < 6 ; i++ ) {
    value[0] = scale->getAverage(6);
  }

  // begin
  for ( i = 0 ; i < readings ; i++ ) {
    //DEBUGPRINT3("=== reading : "); DEBUGPRINTLN3(i);
    sum = 0;
    delay(10);
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    for ( j = 0 ; j < samples ; j++ ) { // fill the table and calculate the average
      while ( !scale->available() );
      value[j] = scale->getReading();
      //DEBUGPRINT3("value:"); DEBUGPRINTLN3(value[j]);
      sum += (double)value[j];
    }
    average[i] = (int32_t)(sum / samples);
    //DEBUGPRINT3("average all samples :"); DEBUGPRINT3(average[i]);
    sum = 0;
    for ( j = 0 ; j < samples ; j++ ) { // standard deviation of samples (ecart type)
      delta = value[j] - average[i];
      sum += (double)(delta * delta);
    }
    s[i] = (int32_t)sqrt(sum / (samples-1));
    //DEBUGPRINT3("  s(ecart type):"); DEBUGPRINTLN3(s[i]);
    sum = 0; k = 0;
    for ( j = 0 ; j < samples ; j++ ) { // average  1 sigma
      //DEBUGPRINTLN3(value[j] - average[i]);
      if ( abs(value[j] - average[i]) < s[i] ) {
        sum += value[j];
        k++;
      }
    }
    average[i] = sum / k;
    sum_ave += average[i];
    //DEBUGPRINT3("average 1 sigma "); DEBUGPRINT3(i); DEBUGPRINT3(" :"); DEBUGPRINT3(average[i]);
    //DEBUGPRINT3(" - k:"); DEBUGPRINTLN3(k);
  }
  DEBUGPRINT3("AVERAGE:"); DEBUGPRINTLN3(sum_ave / i);
  *average_global = sum_ave / i;

  // check if all the readings give the same value
  if ( calibrationFactor[sensor] ) { // measure mass
    for ( i = 0 ; i < readings ; i++ ) { // standard deviation of readings (ecart type)
      delta = abs(average[i] - *average_global)/calibrationFactor[sensor];
      DEBUGPRINT3(" delta="); DEBUGPRINT3(delta);
      DEBUGPRINT3(" - average_global/10="); DEBUGPRINT3(*average_global/10);DEBUGPRINTLN3(" ## ");
      if ( delta > DELTA_MASS_MAX/sensor_nb ) { //abs(*average_global/10) ) {
        Serial.print("\n ERROR ");
        //for ( i = 0 ; i < readings ; i++ ) {DEBUGPRINT3(i);DEBUGPRINT3("="); DEBUGPRINT3(average[i]/calibrationFactor[sensor]);DEBUGPRINT3(" ");}
        return 1;
      }
    
    }
    return 0;    
  } else { //calibration 
    sum = 0;
    for ( i = 0 ; i < readings ; i++ ) { // standard deviation of readings (ecart type)
      delta = average[i] - *average_global;
      sum += (double)(delta * delta);
    }
    //sg = (int32_t)sqrt(sum / (readings-1));
    DEBUGPRINT3("sg:"); DEBUGPRINTLN3(sg);
    if ( sg < abs(*average_global/100) // s (ecart type) < average/100
       || sg <100 ) { // needed for littles average values
      DEBUGPRINT3("ecart type="); DEBUGPRINT3((int32_t)sqrt(sum / (readings-1)));
      DEBUGPRINT3(" - moy/100="); DEBUGPRINT3(*average_global/100);DEBUGPRINT3(" ## ");
      return 0;
    } else { // error
      DEBUGPRINT3("ecart type="); DEBUGPRINT3((int32_t)sqrt(sum / (readings-1)));
      DEBUGPRINT3(" - moy/100="); DEBUGPRINT3(*average_global/100);DEBUGPRINT3(" ## ");
      return 1;
    }    
  }
}
