/* weight scale for bee hives */
/* see : http://rucher.polytech.unice.fr/index.php
 *       https://github.com/christian-peter/ruche-connecte/
 * --------------------------------------------------
 * IMPORTANT : choose "Minimal SPIFFS" in Tools -> Partition scheme
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

// ***********************************************************************
// ** adapt to your config
// ** change default LoRa parameters in esp_lora.h
// choose language
#define LANGUAGE 'F' // 
// uncomment to measure temperature with 18B20 or DHT11/DHT22
#define T_18B20
//#define T_DHT // not yet implemented
// default values
uint16_t sleep_time_day_id   =     0;   // to get the time in minutes
uint16_t sleep_time_night_id =     3;   // see the sleep_time_values below
uint16_t sleep_time_for_visit_id = 1;   // O = 15 minutes, 11 = 1440 mn = 24 hours
long     sleep_time_values[] = {15, 30, 45, 60, 90, 120, 180, 240, 300, 360, 720, 1440}; // minutes
uint8_t  send_max_tries = 4;            // if the first data sent has no acknoledge, number of resend tries, max SEND_MAX_TRIES_MAXI
// don't modify these values unless you understand all the code
#define  CAL_SAMPLES 255 // 16 bits // 4 cells in 1 wheatstone bridge : 500 else 255
#define  CAL_READINGS 15 // 8  bits
#define  SAMPLES  50     // 16 bits // 4 cells in 1 wheatstone bridge : 200 else 50
#define  READINGS 5      // 8  bits
#define  DELTA_MASS_MAX 1000 // g
// ***********************************************************************
const char version[] = " 1.42";  // with fuel gauge
#define MODULE 'S' // G:gateway, S:scale (weight), I:internal, W:weather
 
// debug
#define DEBUGLEVEL 3 // -1 (mute) to 3 (verbose)
//#define TEST_PERIPH // uncomment to test hardware & Lora frequency (LoRa after line 435)
#include <DebugUtils.h>

// time
/*enum param_type {SLEEP_TIME_DAY_PARAM, SLEEP_TIME_NIGHT_PARAM, SLEEP_TIME_FOR_VISIT_PARAM, SEND_MAX_TRIES_PARAM,
                 TXPOWER_PARAM, FREQ_PARAM, SF_PARAM, SBW_PARAM};*/ /* used with OLED settings */
#define FREQ_COEF 3 // to keep the same delay with rtc_clk_cpu_freq modification (RTC_CPU_FREQ_80M -> 240/80 = 3)
#define AFF_TIME  3000/FREQ_COEF  // time the data are visible on the OLED display
#define WAIT_TIME_FOR_CONFIG  5000/FREQ_COEF  // milli seconds
#define DELAY_BEFORE_NEXT_SEND  5/FREQ_COEF      // secondes - MUST BE LOWER THAN 'WDT_TIME_LIMIT'
#define WAIT_TIME_FOR_ACKNOWLEDGE       20000/FREQ_COEF // milli seconds
#define WAIT_TIME_FOR_NEXT_BTN          10000/FREQ_COEF // milliseconds
#define SEND_MAX_TRIES_MAXI 7
uint16_t sleep_time;
long    previousTime = 0;

// sleep - wake up
#define BUTTON_PIN_BITMASK 0x000000004 // 2^2 in hex VALID input
#define uS_TO_MN_FACTOR 60000000ULL  /* Conversion factor for micro seconds to minutes */
//#define uS_TO_MN_FACTOR  6000000ULL  // for test

// datas
uint8_t actions_order[] = {0, 5, 7, 1, 2, 3, 4, 6, 8, 9, 10, 11, 12};
struct datas { // 255 <=> error
  uint8_t macSTA[6];
  float   mass = 255;         // 0 to 200 000 g,
  float   v_bat = 255;        // 0 to 16 V
  uint8_t charg_bat = 255;    // 0 to 100%
  uint8_t action; // 0:no_action,1:Sirup50/50,2:Sirup70/30,3:Candy,4:materiel,5:Visit,6:measure only,7:sleep,8:infos bat,9:version,10:lora,11:temp drift
#if defined T_18B20 || defined T_DHT
  float   temp  = 255;        // -50 to +80 °C
  uint8_t hygro = 255;        //  0 to 100%
#endif
} ds;
RTC_DATA_ATTR float previous_mass;
uint16_t delta_mass = 2000.0;
uint8_t  send_counter = 0;   // max SEND_MAX_TRIES_MAXI,  bits 2 to 0
uint8_t  error_flags  = 0;   // bits 7 to 3
                            // error mass < 0     0b00001000
                            // error mass measure 0b00010000
                            // error temp measure 0b00100000
                            // alarm delta mass   0b01000000
float   vbat, valim, vldr;
float   tbat;
boolean day_state = true;
time_t  now;
RTC_DATA_ATTR time_t  sunrise,sunset;
RTC_DATA_ATTR boolean last_day_state;
double  sunrise_diff_time, sunset_diff_time; 
#define DELAY_AFTER_SUNRISE 1380LL // minutes 23h*60=1380mn
#define DELAY_AFTER_SUNSET    60LL // minutes
uint8_t ui8_tmp;
char    buf_mass[10];

// GPIO
#define MEASURE 13
#define LED_R 33
#define LED_G 25
#define LDR_PIN ADC1_CHANNEL_0 // 36 (PCB V1.10  27)
#define VBAT_PIN  ADC1_CHANNEL_6 //34
#define K_VBAT 0.272 // Vadc/Vbat = 0.265 ; R2/(R1+R2)= 0.27027027 R1=27k R2=10k
// K_VBAT = 0.27 => Vbat = Vextern from 3.3 to 3V ; Vbat = 2.7V @ Vextern = 2.75V ; Vbat = 2.4V @ Vextern = 2.47V
#define VALIM_PIN ADC1_CHANNEL_7 //35
#define K_VALIM 0.052 // Vadc/Valim = 0.06025@12V 0.06012@5V ; R2/(R1+R2)= 0.0625 R1=150k R2=10k
#define VALID_BTN 2   // K_VALIM = 0.052 => Valim = 5V @ Vextern = 5V   ; Valim = 3.65V @ Vextern = 4V 
#define NEXT_BTN  4   //                    Valim = 14V @ Vextern = 12V ; Valim = 12.7V @ Vextern = 11V ; Valim = 11.4V @ Vextern = 10V
#define PROG_BTN 0    // K_VALIM = 0.059 => Valim = 12V @ Vextern = 12V ; Valim = 11.2V @ Vextern = 11V ; Valim = 10.15V@ Vextern = 10V
// LoRa pins
#define SS 18   // 5  - 18 TTGO
#define RST 14
#define DI0 26
// SPI pins
#define SCK 5     // 18 - 5 TTGO
#define MISO 19
#define MOSI 27   // 23 - 27 TTGO
#define CS 18     //5 - 18 TTGO
// I2C pins : NAU7802 N°1 + OLED + battery
#define SDA  21
#define SCL  22
#define DRDY 27  // not used !
// I2C pins : NAU7802 N°2
#define SDA1  17
#define SCL1  16
#define DRDY1 39  // not used !

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <driver/adc.h>
#include <soc/rtc.h>  // for rtc_clk_cpu_freq_set
#include <rom/rtc.h>
#include "EEPROM.h"

#include "esp_lora.h"
#include "global_functions.h"
#include "scale_ble.h"
#include "scale_language.h"
#include "scale_functions.h"
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU8702

// scale / eeprom
bool     init_eeprom_flag = false;
bool     scale_calibration_flag = false;
uint16_t scale_id = 0;
NAU7802  scale0; //Create instance of the NAU7802 class
NAU7802  scale1; //Create instance of the NAU7802 class
enum     sensor {SENSOR_0, SENSOR_1, SENSOR_2, SENSOR_3};
float    calibration_temp = 20.0;
float    temp_factor = 0.0;
float    calibrationFactor[4] = {0.0, 0.0, 0.0, 0.0}; //Value used to convert the load cell reading to lbs or kg
int32_t  zeroOffset[4] = {0, 0, 0, 0};           //Zero value that is found when scale is tared
float    massOffset = 0;  // due to weighing pan 
int32_t  onScale = 0;           
uint8_t  sensor_nb = 0;
uint32_t know_weight = 0; // grams
const uint8_t  gainValue = NAU7802_GAIN_128;        //Set the gain. x1, 2, 4, 8, 16, 32, 64, 128 are available
const uint8_t  ldoValue  = NAU7802_LDO_2V7;
const uint8_t  rate = NAU7802_SPS_320;
const uint8_t  get_ave_max_tries = 5;  // default nb of get average before error flag

// Ticker
#include <Ticker.h>
#define BLINK_TIME_ON    10 //ms
#define BLINK_TIME_OFF  500 //ms
Ticker tickerSetHigh;
Ticker tickerSetLow;
void setPinHigh(int led_pin) { digitalWrite(led_pin, HIGH); }
void setPinLow(int led_pin)  { digitalWrite(led_pin, LOW);  }

// OLED
// need to modify ./libraries/...../OLEDDisplayFonts.h
#include "SH1106.h"  // or #include "SSD1306.h"
SH1106 display(0x3c, SDA, SCL); // or SSD1306 display(0x3c, SDA, SCL);
enum oled_state {OLED_OFF, OLED_ON};

// temperature
int temp_flag = 0 ; // no temperature sensor
#ifdef T_18B20
#include <OneWire.h>
#define ONE_WIRE_BUS 32
OneWire temp_sensor(ONE_WIRE_BUS);
byte temp_data[12];
byte temp_addr[8];
long requestTime;
uint8_t read_temp_tries = 0;
#define MAX_READ_TEMP_TRIES 5
#endif

// LoRa
#define ACKNOWLEDGE_LENGTH 8
#define PAYLOAD_LEN 15
uint8_t payload[PAYLOAD_LEN];
int     packetSize;
bool    myMACaddress = true;

// CRC 
#define CRC_32
#ifdef CRC_32
#include <CRC32.h>
CRC32 crc;
uint32_t checksum;
#endif

// Fuel gauge MAX17263
#include <Streaming.h>
#include "Definitions.h"
#include "Albert_MAX17263.h"
MAX17263 max17263;
byte max17263_present;
bool debug17263 = 1; // print all values for testing the software
history_t history; // todo in EEPROM
// Valeurs de la batterie 
// Cap (mAh) | bool r100 = 1 si NTC > 100k | bool vChg = 1 si tension de charge > 4.25V | 0 : lithium cobalt-oxyde, 2 : pour lithium NCR or NCA, 6 : pour LiFePo
long Cap_ = 7000;
bool r100_ = 0;
bool vChg_ = 0;
uint8_t modelID_ = 6; // LiFePO4

// === Watchdog ==============================
#define WDT_TIME_LIMIT 600000  // 100000 => 20s
hw_timer_t *wdt_timer = NULL; //watchdog timer
void IRAM_ATTR resetModule() {
  ets_printf("Watchdog reboot\n");
  esp_restart();
}

// ***********************************************************************
// ** setup
// ***********************************************************************
void setup() {
  uint8_t i = 0 ;
  uint8_t next = 0;
  uint8_t next_mem = 0;
  uint8_t valid = 0;
  uint8_t valid_mem = 0;
  
  sleep_time = sleep_time_values[sleep_time_day_id];

  // For consumption
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  //RTC_CPU_FREQ_XTAL = 0,      //!< Main XTAL frequency
  //RTC_CPU_FREQ_80M = 1,       //!< 80 MHz
  //RTC_CPU_FREQ_160M = 2,      //!< 160 MHz
  //RTC_CPU_FREQ_240M = 3,      //!< 240 MHz
  //RTC_CPU_FREQ_2M = 4,        //!< 2 MHz ?????????????????????????????
  //(esp_bluedroid_disable(), esp_bt_controller_disable(), esp_wifi_stop())

  // Watchdog
  wdt_timer = timerBegin(0, 80 * 1000, true); // ???? timer 0, div 80*1000 - clock is 80MHZ so every tick will take 1 millisecond
  timerAttachInterrupt(wdt_timer, &resetModule, true);
  timerAlarmWrite(wdt_timer, WDT_TIME_LIMIT, false);
  timerAlarmEnable(wdt_timer); //enable interrupt
  timerWrite(wdt_timer, 0); //reset timer (feed watchdog)

  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE, LOW); // Peripherals power off
  pinMode(LED_R, OUTPUT); digitalWrite(LED_R, LOW);
  pinMode(LED_G, OUTPUT); digitalWrite(LED_G, LOW);
  //pinMode(LDR_PIN,INPUT);

  /* We set our ESP32 to wake up for an external trigger.
    There are two types for ESP32, ext0 and ext1 .
    ext0 uses RTC_IO to wakeup thus requires RTC peripherals to be on while
    ext1 uses RTC Controller so doesnt need peripherals to be powered on.
    Note that using internal pullups/pulldowns also requires RTC peripherals to be turned on. */
  // configure the wake up source
  esp_sleep_enable_ext0_wakeup((gpio_num_t)VALID_BTN, 1); //1 = High, 0 = Low
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  Serial.begin(115200);
  while (!Serial) {;}

  DEBUGPRINT0(INIT_MSG1); DEBUGPRINT0(" "); DEBUGPRINT0(INIT_MSG2); DEBUGPRINTLN0(" Ver. " + (String)version); 
    
  DEBUGPRINT0("DEBUG LEVEL "); DEBUGPRINT3(" 3 "); DEBUGPRINT2(" 2 "); DEBUGPRINT1(" 1 "); DEBUGPRINTLN0(" 0 ");
  DEBUGPRINT3("SAMPLES : "); DEBUGPRINT3(SAMPLES); DEBUGPRINT3(" - READINGS : "); DEBUGPRINTLN3(READINGS); 
  DEBUGPRINT0("TIME Start : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");

  //get_macAdress
  esp_efuse_read_mac(ds.macSTA);
  Serial.println("== mac STA ======");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                ds.macSTA[0], ds.macSTA[1], ds.macSTA[2], ds.macSTA[3], ds.macSTA[4], ds.macSTA[5]);
#if DEBUGLEVEL > 0
  Serial.printf(" = %d:%d:%d:%d:%d:%d\n",
                ds.macSTA[0], ds.macSTA[1], ds.macSTA[2], ds.macSTA[3], ds.macSTA[4], ds.macSTA[5]);
  Serial.print("=================");
#endif
  Serial.println("");

  // eeprom
  if (!EEPROM.begin(EEPROM_SIZE)) {
    DEBUGPRINTLN0("Failed to initialise EEPROM");
    go_to_sleep (sleep_time, OLED_OFF);
  }
  
  // Peripherals power on
  DEBUGPRINT0("TIME Peripherals on : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
  digitalWrite(MEASURE, HIGH); 

  SPI.begin(SCK, MISO, MOSI, CS);
  LoRa.setPins(SS, RST, DI0);
  Wire.begin(SDA, SCL);
  Wire.setClock(400000); //We can increase I2C clock speed to 400kHz, the NAU7802 supports it
  Wire1.begin(SDA1, SCL1);
  Wire1.setClock(400000); //We can increase I2C clock speed to 400kHz, the NAU7802 supports it

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

  DEBUGPRINT0("TIME LoRa payload sent : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
  previousTime = millis();
}

// ***********************************************************************
// ** loop
// ***********************************************************************
void loop() {
  uint8_t i = 0;
  uint8_t p;
  uint8_t ack_OK;


  timerWrite(wdt_timer, 0); //reset timer (feed watchdog)

  // without acknowledge after WAIT_TIME_FOR_ACKNOWLEDGE -> go to sleep
  if ( millis() > previousTime + WAIT_TIME_FOR_ACKNOWLEDGE ) {
    LoRa_stop();
    Serial.print("TIME LoRa stopped (timeout) : "); Serial.print(millis()); Serial.println(" ms ***************");
    digitalWrite(LED_R, HIGH);
    if ( ds.action ) {
      DEBUGPRINT3(NO_ACK1 + String(" ")); DEBUGPRINT3(NO_ACK2 + String(" ")); DEBUGPRINTLN3(NO_ACK3);
      oled_print(OB24, NO_ACK1, NO_ACK2, NO_ACK3, "");
      delay(AFF_TIME);
    } else {
      delay(500); // delay red led on and before sleep to end all process (LoRa)
    }
    go_to_sleep (sleep_time, ds.action);
  }

  // wait acknowledge
  packetSize = LoRa.parsePacket();
  if ( packetSize > 0 ) { // ===== received a packet
    DEBUGPRINT3("Received  size : "); DEBUGPRINTLN3(packetSize);
    if ( packetSize != ACKNOWLEDGE_LENGTH ) {  // ===== received a packet not acknowledge size 
      DEBUGPRINT3("Received packet - size : "); DEBUGPRINTLN3(packetSize);
      LoRa.readString();  // clear the buffer
    } else { // ===== received a acknowledge 
      DEBUGPRINT3("Received packet - acknowledge size : "); DEBUGPRINTLN3(packetSize);
      i = 0;
      myMACaddress = true;
      while (LoRa.available()) { // ===== read the packet and clear the buffer
        p = LoRa.read();
        DEBUGPRINT3(p); DEBUGPRINT3(".");
        if ( p != payload[i] && i < ACKNOWLEDGE_LENGTH -1 ) // 7 first bits
          myMACaddress = false;
        if ( i == ACKNOWLEDGE_LENGTH - 1 ) // CRC bit
          ack_OK = p;
        i++;
      }
      DEBUGPRINTLN3("");
      if ( myMACaddress ) { // ===== an acknowlege for me
        DEBUGPRINTLN3("my mac adress");
        if ( ack_OK == 1 ) {  // ===== acknowledge OK => go to sleep
          LoRa_stop();
          Serial.print("TIME LoRa stopped (ack OK) : "); Serial.print(millis()); Serial.println(" ms ***************");
          // stop blinking led
          tickerSetLow.detach();
          tickerSetHigh.detach();
          digitalWrite(LED_G, HIGH);
          if ( ds.action ) {
            oled_print(OB24, String(buf_mass), "OK", SEND + String(send_counter+1), "");
            delay(AFF_TIME);
          } else {
            delay(500/FREQ_COEF); // delay green led on
          }
          go_to_sleep (sleep_time, ds.action);
        } else {  // ===== acknowledge KO
          DEBUGPRINTLN3("ack KO");
          flash_led(LED_R, 100, 100, send_counter+1);
          if ( send_counter >= send_max_tries ) {  // ===== max tries reached => sleep
            DEBUGPRINTLN3("max tries reached, go to sleep");
            if ( ds.action ) {
              oled_print(OB24, NO_ACK1, NO_ACK2, NO_ACK3, "");
              delay(AFF_TIME);
            } else {
              delay(100); // delay before sleep to end all process (LoRa)
            }
            go_to_sleep (sleep_time, ds.action);
          }
          // =====  send again
          DEBUGPRINTLN3("wait before new send");
          send_counter++;
  
          if (ds.action)
            oled_print(OB24, String(buf_mass), "", SEND + String(send_counter+1), "");
  
          delay(1000 * random(DELAY_BEFORE_NEXT_SEND / 5, DELAY_BEFORE_NEXT_SEND));
  
          LoRa_send_payload(payload, PAYLOAD_LEN, error_flags | send_counter);
  
          previousTime = millis();
        }
      }
    }
  }

}
// ***********************************************************************
// ** end loop
// ***********************************************************************

// ** 18B20 temperature measure *******************************************
#if defined T_18B20
// ******************
void init_temp_sensor() {
  uint8_t i = 0 ;
  
  while ( !temp_sensor.search(temp_addr) && i < 5 ) {
    //DEBUGPRINTLN3("No more addresses.");
    temp_sensor.reset_search();
    delay(250);
    i++;
  }
  
  if (OneWire::crc8(temp_addr, 7) != temp_addr[7] || i > 4 ) {
    DEBUGPRINTLN3("CRC is not valid or no temp sensor!");
    for( i = 0 ; i < 8 ; i++)
      temp_addr[i] = 0;
  }
  
  Serial.print("temp_flag ");Serial.println(temp_flag);
  Serial.print("ROM =");
  for( int j = 0; j < 8; j++) {
    Serial.write(' ');
    Serial.print(temp_addr[j], HEX);
  }
  Serial.println(' ');
  
  // write to eeprom
  for( i = 0 ; i < 8 ; i++)
    EEPROM.put(i+LOCATION_18B20_ROM, temp_addr[i]);

  EEPROM.commit();
}
// ******************
void read_temp_sensor_addr() {
  uint8_t i;
  // 18B20 rom address initialized at 00000000
  
  temp_flag = 0;
  DEBUGPRINT3("Read 18B20 addr : ");  
  for( i = 0 ; i < 8 ; i++) {
    EEPROM.get(i+LOCATION_18B20_ROM, temp_addr[i]);
    DEBUGPRINT3(temp_addr[i]); DEBUGPRINT3(" ");
    temp_flag += temp_addr[i];
  }
  DEBUGPRINTLN3("");  
  DEBUGPRINT3("temp_flag : "); DEBUGPRINTLN3(temp_flag);
}
// ******************
float get_temp() {
  float celsius;
  uint8_t i;
  
  temp_sensor.reset();
  temp_sensor.select(temp_addr);    
  temp_sensor.write(0xBE);         // Read Scratchpad
  
//  Serial.print("  Data = ");
//  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    temp_data[i] = temp_sensor.read();
//    Serial.print(temp_data[i], HEX);
//    Serial.print(" ");
  }
//  Serial.print(" CRC=");
//  Serial.print(OneWire::crc8(temp_data, 8), HEX);
//  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (temp_data[1] << 8) | temp_data[0];
  // the first ROM byte indicates which chip
  if ( temp_addr[0] == 0x10 ) { // Chip = DS18S20") or old DS1820
    raw = raw << 3; // 9 bit resolution default
    if (temp_data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - temp_data[6];
    }
  } else if ( temp_addr[0] == 0x22 || temp_addr[0] == 0x28 ) { // DS1822 or DS18B20
    byte cfg = (temp_data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  } else {
    Serial.print("Device is not a DS18x20 family device.");
    return 255; // error
  }
  celsius = (float)raw / 16.0;
  //DEBUGPRINT3("  Temperature = "); DEBUGPRINT3(celsius); DEBUGPRINTLN3(" °C");
  
  return celsius;
}

// ***********************************************************************
#endif



// ** test devices *******************************************
#if DEBUGLEVEL > 2
void test_devices() {
  
  Serial.println("\nCheck devices ....................");
  // I2C ===========
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C ...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of the Write.endTransmisstion to see if a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.print(" : ");
      switch (address ) {
        case 42 : 
          Serial.println(" NAU7802 N°1 "); break; //0x2A
        case 61 : //0x3D (default) 
        case 60 : //0x3C
          Serial.println(" OLED SH1106 "); break;
        case 54 : 
          Serial.println(" FUEL GAUGE "); break;
        default: Serial.println(" ?"); break;
      }
      nDevices++;
    } 
//    else if (error==4) {
//      Serial.print("Unknown error at address 0x");
//      if (address<16) 
//        Serial.print("0");
//      Serial.println(address,HEX);
//    }    
  }
  address = 42;
  Wire1.beginTransmission(address);
  error = Wire1.endTransmission();
  if (error == 0) {
    Serial.print("I2C device found at address 0x2A : ");
    Serial.println(" NAU7802 N°2 ");
    nDevices++;
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Scanning I2C done");

  // Lora ===========
  if ( LoRa.begin(frequency_values[0]) ) 
    Serial.println("LoRa OK.");
  else
    Serial.println("LoRa init failed. Check your connections.");
  LoRa_stop ();
  Serial.println("END Check devices ....................\n");

}
#endif




/*Création du Hotspot

    Ouvrir Paramètres système
    Dans la partie gauche de la fenêtre Paramètre cliquez sur WiFi.
        En haut de la fenêtre, s'assurer que le WiFi est activé et cliquez la carte WiFi qui n'est pas actuellement connectée à Internet (la clef ou la carte WiFi interne).
        Puis cliquez sur le bouton (en triple trait) qui permet d'afficher plus d’icônes. Dans le menu contextuel, cliquez sur Allumer le point d'accès Wi-FI.
    Validez le message de confirmation qui apparaîtra.
    Le hotspot WiFi sera crée. Vous verrez le nom du partage (i.e. le nom du point d'accès) et le mot de passe. Vous pouvez alors chercher le point d'accès avec vos autres smartphones et appareils et les connecter.
*/
