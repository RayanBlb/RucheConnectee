#include "scale_ble.h"
#include "global_functions.h"

// ==============================
void create_ble_uart (uint8_t * macSTA) {
  // Create the BLE Device
  long previousTimeSec=0;
  char buf[40];

  oled_print(OC16, BLE_CONNECT1, BLE_CONNECT2, BLE_CONNECT3, ""); 

  sprintf(buf,"%s %02X:%02X:%02X:%02X:%02X:%02X",BLE_SERVER,macSTA[0],macSTA[1],macSTA[2],macSTA[3],macSTA[4],macSTA[5]);
  BLEDevice::init(buf);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY);     
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->addDescriptor(new BLE2902());
  
  //pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  
  DEBUGPRINTLN3("BLE service started");
  DEBUGPRINT3("connect with bluetooth to : ");DEBUGPRINTLN3(buf);
  
  // wait BLE connection
  previousTime = millis();
  previousTimeSec = previousTime/1000;
  while (!deviceConnected) {
      if ( millis() > (previousTimeSec*1000)+1000 ) {
        sprintf(buf,"%02X:%02X:%02X:%02X:%02X:%02X",macSTA[0],macSTA[1],macSTA[2],macSTA[3],macSTA[4],macSTA[5]);
        oled_print(OC16, BLE_CONNECT1, BLE_CONNECT2, BLE_CONNECT3 + String("   ") +
        (WAIT_TIME_FOR_BLE + previousTime - millis())/1000 + String("s"), buf); 
        previousTimeSec = millis()/1000;
      }
      delay(10);
      timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
      if ( millis() > previousTime + WAIT_TIME_FOR_BLE ) 
        go_to_sleep (sleep_time, OLED_ON);
  }
  delay(1500); // needed before start !?
  oled_print(OC16, BLE_CONNECT1, BLE_CONNECT2, BLE_CONNECT4,"");
  DEBUGPRINT3("connected to : ");DEBUGPRINTLN3(buf);
}

// ==============================
uint8_t ble_settings_choice() {
  int ans = 0;
  
  previousTime = millis();
  write_to_ble_terminal (SETTING, LINE_FEED_ON);
  write_to_ble_terminal ("1 - ", LINE_FEED_OFF); write_to_ble_terminal (GENERAL, LINE_FEED_ON);
  write_to_ble_terminal ("2 - ", LINE_FEED_OFF); write_to_ble_terminal (LORA, LINE_FEED_ON);
  //write_to_ble_terminal ("3 - ", LINE_FEED_OFF); write_to_ble_terminal (CALIBRATION, LINE_FEED_ON);
  write_to_ble_terminal ("3 - ", LINE_FEED_OFF); write_to_ble_terminal (TEMP_DRIFT, LINE_FEED_ON);
  write_to_ble_terminal ("4 - ", LINE_FEED_OFF); write_to_ble_terminal (RST_EEPROM, LINE_FEED_ON);
  do {
    write_to_ble_terminal (CHOICE, LINE_FEED_ON);
    read_ble_terminal (answer, sizeof(answer));
    ans = atoi(answer);
  } while ( ans < 1 || ans > 4 ); 
  return ans;
}

// ==============================
int ble_select_nb_param (int nb, char *param, uint8_t nb_start, uint16_t nb_stop, uint8_t nb_step, char *units) {
  int     ans = 0;
  char    buf[55];

  do {
    sprintf(buf,"%s (%d %s)", param, nb, units);
    write_to_ble_terminal (buf, LINE_FEED_ON);
    sprintf(buf,"%s [%d-%d]", CHOICE, nb_start, nb_stop);
    write_to_ble_terminal (buf, LINE_FEED_ON);
    read_ble_terminal (answer, sizeof(answer));
    ans = atoi(answer);
  } while ( ans < nb_start || ans > nb_stop );
  sprintf(buf,"%s : %d %s\n", param, ans, units);
  write_to_ble_terminal (buf, LINE_FEED_ON);
  return ans;
}

// ==============================
int ble_select_time_param_id (uint16_t nb_id, char *sleep_time_type) {
  uint8_t   i = 0;
  int       ans = 0;
  char      buf[55];

  do {
    if ( sleep_time_values[nb_id] < 120 )
      sprintf(buf,"%s %s (%d mn)", SLEEP_TIME, sleep_time_type, sleep_time_values[nb_id]);
    else
      sprintf(buf,"%s %s (%d h)", SLEEP_TIME, sleep_time_type, sleep_time_values[nb_id]/60);
    write_to_ble_terminal (buf, LINE_FEED_ON);
    for ( i = 0 ; i < sizeof(sleep_time_values)/sizeof(*sleep_time_values) ; i++ ) {
      if ( sleep_time_values[i] < 120 )
        sprintf(buf,"%4d : %d mn     ", i, sleep_time_values[i] );
      else 
        sprintf(buf,"%4d : %d h     ", i, sleep_time_values[i]/60 );
      if ( i % 2 )
        write_to_ble_terminal (buf, LINE_FEED_ON);
      else
        write_to_ble_terminal (buf, LINE_FEED_OFF);
    }
    sprintf(buf,"%s [%d-%d]", CHOICE, 0, (sizeof(sleep_time_values)/sizeof(*sleep_time_values))-1 );
    write_to_ble_terminal (buf, LINE_FEED_ON);
    read_ble_terminal (answer, sizeof(answer));
    ans = atoi(answer);
  } while ( ans < 0 || ans > (sizeof(sleep_time_values)/sizeof(*sleep_time_values))-1 );
  if ( sleep_time_values[ans] < 120 )
    sprintf(buf,"%s %s : %d mn\n", SLEEP_TIME, sleep_time_type, sleep_time_values[ans]);
  else
    sprintf(buf,"%s %s : %d h\n", SLEEP_TIME, sleep_time_type, sleep_time_values[ans]/60);
  write_to_ble_terminal (buf, LINE_FEED_ON);
  return ans;
}

// ==============================
uint8_t ble_select_freq_param_id (long *values, uint8_t value_nb, uint8_t value_id, char *freq_type) {
  uint8_t   i = 0;
  uint8_t   ans = 0;
  char      buf[55];
  float     coef = 1000000.0;
  char      units[] = "MHz";

  if ( freq_type == LR_SBW ) {
    units[0] = 'k';
    coef = 1000.0;
  }

  do {  
    sprintf(buf,"%s (%.1f %s)", freq_type, values[value_id]/coef, units);
    write_to_ble_terminal (buf, LINE_FEED_ON);
    for ( i = 0 ; i < value_nb ; i++ ) { 
      sprintf(buf,"%4d : %.1f %s     ", i, values[i]/coef, units);
      if ( i % 2 )
        write_to_ble_terminal (buf, LINE_FEED_ON);
      else
        write_to_ble_terminal (buf, LINE_FEED_OFF);
    }
    if ( i % 2 )
      write_to_ble_terminal ("", LINE_FEED_ON);
    sprintf(buf,"%s [%d-%d]", CHOICE, 0, value_nb );
    write_to_ble_terminal (buf, LINE_FEED_ON);
    read_ble_terminal (answer, sizeof(answer));
    ans = atoi(answer);
  } while ( ans < 0 || ans > value_nb );
  sprintf(buf,"%s : %.1f %s\n", freq_type, values[ans]/coef, units);
  write_to_ble_terminal (buf, LINE_FEED_ON);
  
  return ans;
}

// ==============================
void ble_settings_general() {
  char      buf[55];

  sleep_time_day_id       = (uint16_t) ble_select_time_param_id (sleep_time_day_id, SLEEP_TIME_DAY);
  sleep_time_night_id     = (uint16_t) ble_select_time_param_id (sleep_time_night_id, SLEEP_TIME_NIGHT);
  sleep_time_for_visit_id = (uint16_t) ble_select_time_param_id (sleep_time_for_visit_id, SLEEP_TIME_VISIT);
  send_max_tries          = (uint8_t)  ble_select_nb_param (send_max_tries, SEND_MAX_TRIES_NB, 1, SEND_MAX_TRIES_MAXI, 1, "");
  scale_id                = (uint16_t) ble_select_nb_param (scale_id, SCALE_INDEX, 1, 65535, 1, "");
  delta_mass              = (uint16_t) ble_select_nb_param (delta_mass, DELTA_M, 1, 65535, 1, "g");
  // save settings
  EEPROM.put(LOCATION_SLEEP_TIME_DAY, sleep_time_day_id);
  EEPROM.put(LOCATION_SLEEP_TIME_NIGHT, sleep_time_night_id);
  EEPROM.put(LOCATION_SLEEP_TIME_FOR_VISIT, sleep_time_for_visit_id);
  EEPROM.put(LOCATION_SEND_MAX_TRIES, send_max_tries);
  EEPROM.put(LOCATION_SCALE_ID, scale_id);
  EEPROM.put(LOCATION_DELTA_MASS, delta_mass);
  EEPROM.commit();    
  sprintf(buf,"%s %s", SETTING, SAVED);
  write_to_ble_terminal (buf, LINE_FEED_ON);
  write_to_ble_terminal (LOGOUT, LINE_FEED_ON);
}

// ==============================
void ble_settings_lora(struct LoRa_param_s  LR) {
  char      buf[55];
  
  write_to_ble_terminal (LORA, LINE_FEED_ON);
  LR.frequency_id = ble_select_freq_param_id (frequency_values,
    (sizeof(frequency_values)/sizeof(*frequency_values)), LR.frequency_id, LR_FREQ);
  LR.spreadingFactor = ble_select_nb_param (LR.spreadingFactor, LR_SF, 7, 12, 1, "");
  LR.signalBandwidth_id = ble_select_freq_param_id (signalBandwidth_values,
    (sizeof(signalBandwidth_values)/sizeof(*signalBandwidth_values)), LR.signalBandwidth_id, LR_SBW);
  LR.txPower = ble_select_nb_param (LR.txPower, LR_TXPOWER, 2, 15, 1, "dB");
 
  // save settings
  EEPROM.put(LOCATION_LORA_TXPOWER, LR.txPower);
  EEPROM.put(LOCATION_LORA_FREQUENCY, LR.frequency_id);
  EEPROM.put(LOCATION_LORA_SPREADINGFACTOR, LR.spreadingFactor);
  EEPROM.put(LOCATION_LORA_SIGNALBANDWIDTH, LR.signalBandwidth_id);
  //EEPROM.put(LOCATION_LORA_SYNCWORD, LoRa_param_1.syncWord);
  //EEPROM.put(LOCATION_LORA_OCP, );
  EEPROM.commit();    
  sprintf(buf,"%s %s", SETTING, SAVED);
  write_to_ble_terminal (buf, LINE_FEED_ON);
  write_to_ble_terminal (LOGOUT, LINE_FEED_ON);
}

// ==============================
void ble_settings_temperature_drift() {
  uint8_t    i = 0;
  float      ans = 0;
  char       buf[55];
  
  do {
    sprintf(buf,"%s (%.1f %s)", TEMP_DRIFT, temp_factor, "g/°C");
    write_to_ble_terminal (buf, LINE_FEED_ON);
    sprintf(buf,"%s [%.1f : %+.1f]", CHOICE, -999.9, +999.9);
    write_to_ble_terminal (buf, LINE_FEED_ON);
    read_ble_terminal (answer, sizeof(answer));
    for ( i = 0 ; i < strlen(answer) ; i++ ) {
      //Serial.println(answer[i]);
      if ( answer[i] == ',' )
        answer[i] = '.';
    }
    ans = atof(answer);
  } while ( ans < -999.0 || ans > 999.9 );
  sprintf(buf,"%s : %.1f %s\n", TEMP_DRIFT, ans, "g/°C");
  write_to_ble_terminal (buf, LINE_FEED_ON);

  // save settings
  temp_factor = round(10.0*ans)/10.0; // only 1 decimal
  EEPROM.put(LOCATION_TEMP_FACTOR,temp_factor); 
  EEPROM.commit();
  sprintf(buf,"%s %s", SETTING, SAVED);
  write_to_ble_terminal (buf, LINE_FEED_ON);
  write_to_ble_terminal (LOGOUT, LINE_FEED_ON);
}

// ==============================
void write_to_ble_terminal (char * text, bool line_feed) {
  
  pTxCharacteristic->setValue(text); 
  pTxCharacteristic->notify(); // send 'text' to the terminal
  DEBUGPRINT3(text);
  if ( line_feed ) {
    DEBUGPRINTLN3("");
    pTxCharacteristic->setValue("\n"); 
    pTxCharacteristic->notify();
  }
}

// ==============================
void read_ble_terminal ( char *param, uint8_t len_max) {
  
  std::string rxValue;
  uint8_t strTooLong = 0;
  uint8_t i = 0;
  uint8_t i_param = 0;
  
  for (i = 0; i < len_max ; i++) // reset param
      param[i] = '\0';
  
  do {
    strTooLong = 0;

    do 
      rxValue = pRxCharacteristic->getValue();
    while(rxValue.length() == 0); // wait answer  
    
    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)  

    DEBUGPRINT3("Received Value : "); DEBUGPRINT3(rxValue.c_str()); DEBUGPRINT3("length: "); DEBUGPRINTLN3(rxValue.length());
    for (i = 0, i_param = 0; i < rxValue.length() ; i++) {
      if ( !iscntrl(rxValue[i]) ) { 
        if ( i_param < len_max ) { // remove CR, LF, ....
          param[i_param] = rxValue[i];
          i_param++;
        }
      }
      if ( i_param >= len_max ) {
        strTooLong++;
        i_param--;
      }
    }
    param[i_param] = '\0'; //'\0';
    DEBUGPRINT3("read ble string : ");DEBUGPRINT3(param);DEBUGPRINT3(", length: ");DEBUGPRINTLN3(strlen(param));
    DEBUGPRINTLN3("\n");
      
    if (strTooLong) { 
      write_to_ble_terminal(STRING_TO_LONG,LINE_FEED_ON); 
      pRxCharacteristic->setValue(""); //  reset RxCharacteristic value
    }
  } while ( strTooLong );
 
  pRxCharacteristic->setValue(""); // reset RxCharacteristic value
}
