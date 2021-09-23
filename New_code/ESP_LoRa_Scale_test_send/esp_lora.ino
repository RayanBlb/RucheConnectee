/* weight scale for bee hives */
/* see : http://rucher.polytech.unice.fr/index.php
 *       https://github.com/christian-peter/ruche-connecte/
 * --------------------------------------------------
 * IMPORTANT : choose "Huge APP" in Tools -> Partition scheme
 * --------------------------------------------------
 * LICENCE
 * All rights reserved. 
 * This program and the accompanying materials are made available under the terms of the MIT License 
 * which accompanies this distribution, and is available at https://opensource.org/licenses/mit-license.php 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * --------------------------------------------------
 * Christian.PETER_at_univ-cotedazur.fr
 */
 
 #include "global_functions.h"

//*** start LoRa **********************************************************
void LoRa_start(struct LoRa_param_s  LR) {
  if (!LoRa.begin(frequency_values[LR.frequency_id]+LR.frequency_offset)) {
    DEBUGPRINTLN2("LoRa init failed. Check your connections.");
    oled_print(OB24, "LoRa init", "KO", "connect° ?" , "");
    delay(3000);
    #if( MODULE == 'G')
    while(1);
    #else
    go_to_sleep (sleep_time_values[sleep_time_night_id], OLED_OFF);
    #endif
  }

  DEBUGPRINTLN2("LoRa Initial OK!");
  DEBUGPRINT3("LoRa Frequency: "); DEBUGPRINTLN3(frequency_values[LR.frequency_id]);
  DEBUGPRINT3("LoRa Spreading Factor: "); DEBUGPRINTLN3(LR.spreadingFactor);
  LoRa.setSpreadingFactor(LR.spreadingFactor);
  DEBUGPRINT3("LoRa Signal Bandwidth: "); DEBUGPRINTLN3(signalBandwidth_values[LR.signalBandwidth_id]);
  LoRa.setSignalBandwidth(signalBandwidth_values[LR.signalBandwidth_id]);
  DEBUGPRINT3("LoRa codingRateDenominator: "); DEBUGPRINTLN3(LR.codingRateDenominator);
  LoRa.setCodingRate4(LR.codingRateDenominator);
  DEBUGPRINT3("LoRa preambleLength: "); DEBUGPRINTLN3(LR.preambleLength);
  LoRa.setPreambleLength(LR.preambleLength);  
//  DEBUGPRINT3("LoRa syncWord: "); DEBUGPRINTLN3(LR.syncWord);
//  LoRa.setSyncWord(LR.syncWord);  
  DEBUGPRINT3("LoRa txPower: "); DEBUGPRINTLN3(LR.txPower);
  LoRa.setTxPower(LR.txPower);
  DEBUGPRINTLN3("LoRa enable CRC ");
  LoRa.enableCrc();  
}

//*** payload LoRa ********************************************************
void LoRa_send_payload(uint8_t *payload, uint8_t payload_len, uint8_t check_byte) {
  DEBUGPRINT2("LoRa send:");
  LoRa.beginPacket();
  for (uint8_t i = 0; i < payload_len; i++ ) {
    LoRa.write(payload[i]);
    DEBUGPRINT2(payload[i]);DEBUGPRINT2(".");
  }
  LoRa.write(check_byte);
  DEBUGPRINTLN2(check_byte);
  
  #ifdef CRC_32
  checksum = CRC32::calculate(payload, payload_len);
  DEBUGPRINT2("crc = "); DEBUGPRINT2(checksum); DEBUGPRINT2(" = "); 
  for (size_t j = 0; j < sizeof(checksum); j++) {
    // Convert to unsigned char* because a char is 1 byte in size. That is guaranteed by the standard.
    unsigned char b = *((unsigned char *)&checksum + j);
    LoRa.write(b); 
    DEBUGPRINT2(b); DEBUGPRINT2(".");
  }
  DEBUGPRINTLN2("");
  #endif
  
  LoRa.endPacket();
}

//*** stop LoRa ********************************************************
void LoRa_stop () {
  LoRa.sleep();
  LoRa.end();
}

//*** LoRa Frequency **********************************************************
// some RFM95 have bad frequencies. This routine send Lora datas with differents
// frequencies and wait an acknowledge from the gateway
#if DEBUGLEVEL > 2
void scanLoRaFrequency(long f_start, long f_stop, long f_step, long f_offset) {
  uint8_t i = 0;
  uint8_t p;
  uint8_t ack_OK;
  int f_found = 0 ;
  int f_scanned = 0 ;

  Serial.println("Scanning LoRa ....................");

  ds.mass = 123320;         // 0 to 200 000 g,
  ds.v_bat = 121;        // 0 to 16 V
  ds.charg_bat = 255;    // 0 to 100%
  ds.action = 5; // 0:no_action,1:Sirup50/50,2:Sirup70/30,3:Candy,4:materiel,5:Visit,6:measure only,7:sleep,8:infos bat,9:version,10:lora,11:temp drift
  ds.temp  = 25.5;        // -50 to +80 °C
  ds.hygro = 255;        //  0 to 100%
  error_flags =0;
  send_counter = 0;

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

  // scan frequencies
  for (long f=f_start ; f<f_stop ; f+=f_step ) { //f=863200000 ; f<868100001; f+= 12500

    timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
    
    oled_print(OB24, "Scanning", "LoRa", (String)(f+f_offset), "Hz");

    f_scanned++ ;

    // LoRa start
    if (!LoRa.begin(f+f_offset)) {
      DEBUGPRINTLN2("LoRa init failed. Check your connections.");
      delay(300);
    }
    LoRa.setSpreadingFactor(11); //LoRa_param_1.spreadingFactor);
    LoRa.setSignalBandwidth(125000); //signalBandwidth_values[LoRa_param_1.signalBandwidth_id]);
    LoRa.setCodingRate4(LoRa_param_1.codingRateDenominator);
    LoRa.setPreambleLength(LoRa_param_1.preambleLength);  
    LoRa.setTxPower(LoRa_param_1.txPower);
    LoRa.enableCrc();  
    
    // send data
    Serial.printf("f=%d + %d = %d Hz\n", f, f_offset, f+ f_offset);
    LoRa_send_payload(payload, i, error_flags | send_counter);
    
    // wait ack
    previousTime = millis();
    while ( millis() < previousTime + WAIT_TIME_FOR_ACKNOWLEDGE ) {
      // wait acknowledge
      packetSize = LoRa.parsePacket();
      if ( packetSize > 0 ) { // ===== received a packet
        //DEBUGPRINT3("Received  size : "); DEBUGPRINTLN3(packetSize);
        if ( packetSize != ACKNOWLEDGE_LENGTH ) {  // ===== received a packet not acknowledge size 
          //DEBUGPRINT3("Received packet - size : "); DEBUGPRINTLN3(packetSize);
          LoRa.readString();  // clear the buffer
        } else { // ===== received a acknowledge 
          //DEBUGPRINT3("Received packet - acknowledge size : "); DEBUGPRINTLN3(packetSize);
          i = 0;
          myMACaddress = true;
          while (LoRa.available()) { // ===== read the packet and clear the buffer
            p = LoRa.read();
            //DEBUGPRINT3(p); DEBUGPRINT3(".");
            if ( p != payload[i] && i < ACKNOWLEDGE_LENGTH -1 ) // 7 first bits
              myMACaddress = false;
            if ( i == ACKNOWLEDGE_LENGTH - 1 ) // CRC bit
              ack_OK = p;
            i++;
          }
          //DEBUGPRINTLN3("");
          if ( myMACaddress ) { // ===== an acknowlege for me
            //DEBUGPRINTLN3("my mac adress");
            Serial.print("ACK OK, ");Serial.printf("RSSI %d dBm, SNR %d dBm\n",LoRa.packetRssi(),LoRa.packetSnr());
            f_scanned = 0; f_found++;
          }
        } // ===== received a acknowledge
      } // ===== received a packet
      if ( f_found != 0 && f_scanned > 2 ) {
        oled_print(OB24, "Found", "LoRa", (String)f_found, "");
        Serial.println("\n End scanning ....................");
        while (1) {
          timerWrite(wdt_timer, 0); //reset timer (feed watchdog)
          delay(1000);
        }
      }
    } // while
    LoRa_stop();
    Serial.println("\n");
  }
  
}
#endif
