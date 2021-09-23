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

#include "OLEDDisplayFonts.h"
#include "global_functions.h"

//*** Sleep  **********************************************************
void go_to_sleep (int sleep_time, uint8_t print_oled) {

  esp_sleep_enable_timer_wakeup(sleep_time * uS_TO_MN_FACTOR);
  DEBUGPRINT1(GO_TO_SLEEP);
  DEBUGPRINT1(sleep_time);
  DEBUGPRINTLN1(MINUTES);
  if ( print_oled ) {
    oled_print(OB24, GO_TO_SLEEP1, GO_TO_SLEEP2, String(sleep_time)+MN, "");
    delay(AFF_TIME);
  } else {
    delay(100/FREQ_COEF); // delay before sleep to end all process (LoRa) 
  }
  tickerSetLow.detach();
  tickerSetHigh.detach();

  // from https://www.savjee.be/2019/12/esp32-tips-to-increase-battery-life/
  //WiFi.disconnect(true);
  //WiFi.mode(WIFI_OFF);
  btStop();
  adc_power_off();
  //esp_wifi_stop();
  //esp_bt_controller_disable();

  // Peripherals power off
  DEBUGPRINT0("TIME Peripherals off : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
  digitalWrite(MEASURE, LOW); 
  //pinMode(MEASURE, INPUT_PULLDOWN);
  
  Serial.print("TIME esp_deep_sleep_start : "); Serial.print(millis()); Serial.println(" ms ***************");

  esp_deep_sleep_start(); 
}

//*** Led **********************************************************
void flash_led(int led_gpio, int time_on, int time_off, int repeat) {
  for (uint8_t i = 0; i < repeat ; i++ ) { 
      digitalWrite(led_gpio, HIGH); 
      delay(time_on);
      digitalWrite(led_gpio, LOW);
      delay(time_off);
  }
}

//*** OLED **********************************************************
/* available fonts
OB10 Open_Sans_Bold_10
OB16 Open_Sans_Bold_16
OB24 Open_Sans_Bold_24
OB36 Open_Sans_Bold_36
OC10 Open_Sans_Condensed_Bold_10
OC16 Open_Sans_Condensed_Bold_16
OC24 Open_Sans_Condensed_Bold_24
OC36 Open_Sans_Condensed_Bold_36
OC44 Open_Sans_Condensed_Bold_44
AP10 ArialMT_Plain_10
AP16 ArialMT_Plain_16
AP24 ArialMT_Plain_24 */

void oled_print(enum fonts font, String l0, String l1, String l2, String l3) {
  
//  digitalWrite(OLED_RST, LOW); // set OLEDreset low to reset OLED
//  delay(10);
//  digitalWrite(OLED_RST, HIGH);
  display.init();
  //display.flipScreenVertically();
  display.clear();
  display.setBrightness(255);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  switch ( font ) {
    case OB10 :
      display.setFont(Open_Sans_Bold_10);
      display.drawString(0,0,l0);
      display.drawString(0,15,l1);
      display.drawString(0,30,l2);
      display.drawString(0,45,l3);
      break; 
    case OC10 :
      display.setFont(Open_Sans_Condensed_Bold_10);
      display.drawString(0,0,l0);
      display.drawString(0,15,l1);
      display.drawString(0,30,l2);
      display.drawString(0,45,l3);
      break;
    case OB16 :
      display.setFont(Open_Sans_Bold_16);
      display.drawString(0,0,l0);
      display.drawString(0,15,l1);
      display.drawString(0,30,l2);
      display.drawString(0,45,l3);
      break;
    case OC16 :
      display.setFont(Open_Sans_Condensed_Bold_16);
      display.drawString(0,0,l0);
      display.drawString(0,15,l1);
      display.drawString(0,30,l2);
      display.drawString(0,45,l3);
      break;
    case OB24 :
      display.setFont(Open_Sans_Bold_24);
      display.drawString(0,-5,l0);
      display.drawString(0,16,l1);
      display.drawString(0,37,l2);
      break;
    case OC24 :
      display.setFont(Open_Sans_Condensed_Bold_24);
      display.drawString(0,-5,l0);
      display.drawString(0,16,l1);
      display.drawString(0,37,l2);
      break;
    case OB36 :
      display.setFont(Open_Sans_Bold_36);
      display.drawString(0,-7,l0);
      display.drawString(0,23,l1);
      break;
    case OC36 :
      display.setFont(Open_Sans_Condensed_Bold_36);
      display.drawString(0,-7,l0);
      display.drawString(0,23,l1);
      break;
    case OC44 :
      display.setFont(Open_Sans_Condensed_Bold_44);
      display.drawString(0,0,l0);
      break;
    case AP10 :
      display.setFont(ArialMT_Plain_10);
      display.drawString(0,0,l0);
      display.drawString(0,15,l1);
      display.drawString(0,30,l2);
      display.drawString(0,45,l3);
      break;
    case AP16 :
      display.setFont(ArialMT_Plain_16);
      display.drawString(0,0,l0);
      display.drawString(0,22,l1);
      display.drawString(0,44,l2);
      break;
    case AP24 :
      display.setFont(ArialMT_Plain_24);
      display.drawString(0,2,l0);
      display.drawString(0,33,l1);
      break;
    case OC16B2 :
      display.setFont(Open_Sans_Condensed_Bold_16);
      display.drawString(0,0,l0);
      display.setFont(Open_Sans_Bold_24);
      display.drawString(0,14,l1);
      display.setFont(Open_Sans_Bold_10);
      display.drawString(0,41,l2);
      display.drawString(0,52,l3);
      break;
    default :
      display.setFont(ArialMT_Plain_10);
      display.drawString(0,0,"BAD FONT");
      display.drawString(0,20,"SELECTED"); 
    }
  display.display();
}
