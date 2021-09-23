  //get_macAdress
  esp_efuse_read_mac(ds.macSTA);
  Serial.println("== mac STA ======");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",ds.macSTA[0], ds.macSTA[1], ds.macSTA[2], ds.macSTA[3], ds.macSTA[4], ds.macSTA[5]);

// build payload
  DEBUGPRINTLN2("----------");
  DEBUGPRINT2("mass : "); DEBUGPRINTLN2(buf_mass);
  DEBUGPRINT2("v_bat : "); DEBUGPRINT2(ds.v_bat); DEBUGPRINTLN2(" V");
  DEBUGPRINT2("charg_bat : "); DEBUGPRINT2(ds.charg_bat); DEBUGPRINTLN2(" %");
  DEBUGPRINT2("action : "); DEBUGPRINT2(ds.action); DEBUGPRINT2(" ("); DEBUGPRINT2(action_text[ds.action][0]);
  DEBUGPRINT2(" "); DEBUGPRINT2(action_text[ds.action][1]); DEBUGPRINTLN2(")");
  DEBUGPRINT2("temp : "); DEBUGPRINT2(ds.temp); DEBUGPRINTLN2(" °C");
  DEBUGPRINT2("hygro : "); DEBUGPRINT2(ds.hygro); DEBUGPRINTLN2(" %");
  DEBUGPRINTLN2("----------");
  
  // payload :
  // S.mac_adress(6).Mkg.Mdg.Vbat.ChBat.action.T°.T/100.hygro.error|send_counter
  payload[0] = MODULE;
  DEBUGPRINT3("payload : "); DEBUGPRINT3(payload[0]); DEBUGPRINT3(";");
  for (i = 1 ; i < 7 ; i++) {
    payload[i] = ds.macSTA[i - 1];
    DEBUGPRINT3(payload[i]); DEBUGPRINT3(":");
  }
  payload[i++] = uint8_t(ds.mass/1000); DEBUGPRINT3(uint8_t(ds.mass/1000)); // kg
  ui8_tmp = uint8_t(((ds.mass/1000) - uint8_t(ds.mass/1000)) * 100); // decigrams
  payload[i++] = ui8_tmp; DEBUGPRINT3("."); DEBUGPRINT3(ui8_tmp);
  payload[i++] = uint8_t(ds.v_bat * 10); DEBUGPRINT3(";"); DEBUGPRINT3(uint8_t(ds.v_bat * 10)); // 1/10V
  payload[i++] = ds.charg_bat; DEBUGPRINT3(";"); DEBUGPRINT3(ds.charg_bat); // %
  payload[i++] = ds.action; DEBUGPRINT3(";"); DEBUGPRINT3(ds.action);
  ds.temp += 100;
  payload[i++] = uint8_t(ds.temp); DEBUGPRINT3(";"); DEBUGPRINT3(uint8_t(ds.temp)); // °C + 100
  ui8_tmp = uint8_t( round( ( ds.temp - uint8_t(ds.temp) ) * 100) ); // 1/1OO °C
  payload[i++] = ui8_tmp; DEBUGPRINT3("."); DEBUGPRINT3(ui8_tmp);
  ds.temp -= 100;
  payload[i++] = ds.hygro; DEBUGPRINT3(";"); DEBUGPRINT3(ds.hygro); // %
  DEBUGPRINT3(" payload_len : "); DEBUGPRINTLN3(i);
  DEBUGPRINTLN3("----------");

  // LoRa
  DEBUGPRINT0("TIME LoRa start : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
  LoRa_start(LoRa_param_1);
  DEBUGPRINT0("TIME LoRa started : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");

  LoRa_send_payload(payload, i, error_flags | send_counter);