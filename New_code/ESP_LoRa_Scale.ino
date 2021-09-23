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
#define DEBUGLEVEL -1 // -1 (mute) to 3 (verbose)
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
#define SS 5   // 5  - 18 TTGO
#define RST 14
#define DI0 26
// SPI pins
#define SCK 18     // 18 - 5 TTGO
#define MISO 19
#define MOSI 23   // 23 - 27 TTGO
#define CS 5     //5 - 18 TTGO
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

  // reset ?
  switch (rtc_get_reset_reason(0) ) {
    case EXT_CPU_RESET : // <14, for APP CPU, reseted by PRO CPU
      DEBUGPRINTLN3("reset_reason : EXT_CPU_RESET");
    case RTCWDT_RTC_RESET :  // <16, RTC Watch dog reset digital core and rtc module
      DEBUGPRINTLN3("reset_reason : RTCWDT_RTC_RESET");
    case POWERON_RESET : // power on, RST reset -> print init message on OLED and go to sleep
      DEBUGPRINTLN3("reset_reason : POWERON_RESET");
      EEPROM.get(LOCATION_SCALE_ID, scale_id);
      oled_print(OB24, INIT_MSG1, "N° " + (String)scale_id, "Ver. " + (String)version , "");
      previousTime = millis();
      while ( millis() < WAIT_TIME_FOR_CONFIG + previousTime ) {
        if ( !digitalRead(PROG_BTN) ) {   // if PROG_BTN during init message => reinitialize eeprom & read 18B20 ROM address
          DEBUGPRINTLN2("INIT EEPROM");
          init_eeprom();
          #ifdef T_18B20
          init_temp_sensor();
          #endif
          oled_print(OB24, INIT_SYS1, INIT_SYS2, INIT_SYS3, "");
          delay(AFF_TIME);
          go_to_sleep (sleep_time, OLED_ON);
        }
      }
      go_to_sleep (sleep_time, OLED_ON);
      break;
    case SW_CPU_RESET : // watchdog reset
      DEBUGPRINTLN3("reset_reason : SW_CPU_RESET");
      go_to_sleep (sleep_time_values[sleep_time_night_id], 0);
      break;
    default :
      break;
  }

  // eeprom : if the 6 first byte are the mac address then the eeprom has been initialized
  init_eeprom_flag = true;
  for( i = LOCATION_MAC_ADDRESS ; i < LOCATION_MAC_ADDRESS+6 ; i++) {
    EEPROM.get(i, ui8_tmp);
    if ( ui8_tmp != ds.macSTA[i] ) {
      init_eeprom_flag = false;
      break;
    }
  }
  if ( !init_eeprom_flag ) {
    init_eeprom();
    #ifdef T_18B20
    init_temp_sensor();
    #endif
  }
  read_eeprom();
  #ifdef T_18B20
  read_temp_sensor_addr();
  #endif

  // N° scale
  DEBUGPRINT0(INIT_MSG1); DEBUGPRINT0(" N°"); DEBUGPRINTLN0(scale_id); 
          
  // check if the scale is calibrated
  if ( calibrationFactor[0] )  // has been read in the eeprom - not nul if scale calibrated
    scale_calibration_flag = true;
  else
    scale_calibration_flag = false;

  // blink at startup
  flash_led(LED_G, 200, 0, 1); 

  // temperature : reset the sensor and request conversion
#ifdef T_18B20
  if ( temp_flag ) {
    temp_sensor.reset();
    temp_sensor.select(temp_addr);
    temp_sensor.write(0x44, 0);        // start conversion, Write a byte
    //temp_sensor.write(0x44, 1);        // start conversion, Write a byte and leave power applied to the 1 wire bus.
    DEBUGPRINT0("TIME request temperature : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
    requestTime = millis();
  }
#endif

  // Start measures
  DEBUGPRINT0("TIME Start measures : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");

  // measure power and LDR voltages
  adc_power_on();
  adc1_config_width(ADC_WIDTH_BIT_12);
  valim = get_voltage((adc1_channel_t)VALIM_PIN, ADC_ATTEN_0db , 20, (float)K_VALIM);
  DEBUGPRINT3("Valim  = "); DEBUGPRINT3(valim); DEBUGPRINTLN3(" V");
  vbat  = get_voltage((adc1_channel_t)VBAT_PIN , ADC_ATTEN_0db , 20, (float)K_VBAT );
  DEBUGPRINT3("Vbat  = "); DEBUGPRINT3(vbat); DEBUGPRINTLN3(" V");
  vldr  = get_voltage((adc1_channel_t)LDR_PIN  , ADC_ATTEN_11db , 20, (float)1 );
  DEBUGPRINT3("Vldr  = "); DEBUGPRINT3(vldr); DEBUGPRINTLN3(" V");
  // define day & night sleep time
  time(&now);
  DEBUGPRINT3("time now = "); DEBUGPRINTLN3(now);
  if ( vldr < (vbat * 0.8) ) { // day
    day_state = true;
    if ( !last_day_state ) {
      time(&sunrise);
      DEBUGPRINT3("sunrise  = "); DEBUGPRINT3(sunrise); DEBUGPRINTLN3(" s");
    }
    sleep_time = sleep_time_values[sleep_time_day_id];  
    } else { // night
    day_state = false;
    if ( last_day_state ) {
      time(&sunset);
      DEBUGPRINT3("sunset  = "); DEBUGPRINT3(sunset); DEBUGPRINTLN3(" s");
    }
    sunrise_diff_time = difftime( now, sunrise)/60; //difftime(time_t end, time_t beginning) seconds
    sunset_diff_time = difftime( now, sunset)/60;   // /60 => minutes
    DEBUGPRINT3("sunrise "); DEBUGPRINT3(sunrise); DEBUGPRINT3(", delay "); DEBUGPRINTLN3(sunrise_diff_time);
    DEBUGPRINT3("sunset  ");  DEBUGPRINT3(sunset); DEBUGPRINT3(", delay "); DEBUGPRINTLN3(sunset_diff_time);
    if ( sunrise_diff_time > DELAY_AFTER_SUNRISE || sunset_diff_time < DELAY_AFTER_SUNSET )
      sleep_time = sleep_time_values[sleep_time_day_id];
    else
      sleep_time = sleep_time_values[sleep_time_night_id];
  }
  DEBUGPRINT3("sleep_time = "); DEBUGPRINTLN3(sleep_time);
  DEBUGPRINT3("day_state       = "); DEBUGPRINT3(day_state); DEBUGPRINTLN3("");
  DEBUGPRINT3("last_day_state  = "); DEBUGPRINT3(last_day_state); DEBUGPRINTLN3("");
  last_day_state = day_state;
  
  // if power is external use valim else use vbat
  if ( valim > vbat ) {
    ds.v_bat = valim;
  } else {
    ds.v_bat = vbat;
    ds.charg_bat =  get_charg_bat();
  }
  DEBUGPRINT3("ds.vbat  = "); DEBUGPRINT3(ds.v_bat); DEBUGPRINTLN3(" V");

  // test devices for hardware debug
  #ifdef TEST_PERIPH
  test_devices();
  //scanLoRaFrequency(863700000, 868100001, 12500, LoRa_param_1.frequency_offset); //f_start, f_stop, f_step);//f=863200000 ; f<868100001; f+= 12500
  scanLoRaFrequency(863750000, 868100001, 12500, 0);
  #endif
  
  // Fuel gauge MAX17263
  Wire.beginTransmission(54); // MAX17263 I2C address
  max17263_present = Wire.endTransmission();
  if (max17263_present == 0) {
    if(debug17263) {
      printMultipliers();
    }
    setMax17263memberVariables(Cap_, r100_, vChg_, modelID_); 
    if(max17263.batteryPresent()){ 
        manageMAX17263init(); // Values are stored in RAM, so they must be written any time the power is applied or restored to the device
        printFuelGaugeResults(); // Step 3.3 read the Fuel-Gauge Results 
        ds.charg_bat = max17263.getSOC();
        ds.v_bat     = max17263.getVcell();
        tbat         = max17263.getTemp();
        DEBUGPRINT3("SOC  (fuel gauge)  = "); DEBUGPRINT3(ds.charg_bat); DEBUGPRINTLN3(" %");
        DEBUGPRINT3("Vbat (fuel gauge)  = "); DEBUGPRINT3(ds.v_bat); DEBUGPRINTLN3(" V");
        DEBUGPRINT3("Temp (fuel gauge)  = "); DEBUGPRINT3(tbat); DEBUGPRINTLN3(" °C");
        //manageMAX17263historyRead(); // do at certain intervals
    }
  }
    
  // wake up
  if ( esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0 ) { // wake up from VALID button
    DEBUGPRINTLN2("wakeup cause : ESP_SLEEP_WAKEUP_EXT0");
    oled_print(OC16, BLE_INVIT1, BLE_INVIT2, BLE_INVIT3, SETTING);
    previousTime = millis(); 

    while ( digitalRead(VALID_BTN) ) {  // choose setting mode ************
      /* settings with BLE */
      //DEBUGPRINTLN2(millis());
      if ( millis() > WAIT_TIME_FOR_CONFIG + previousTime ) {  // enter settings mode after WAIT_TIME_FOR_CONFIG
        oled_print(OB16, "CALIBRATION", "<--", SETTING, "         -->");
        while ( digitalRead(VALID_BTN) ) { // wait VALID_BTN to be released
            delay(100);
        } 
        i = 0;
        do {
          if ( digitalRead(VALID_BTN) ) { // go to calibration
            DEBUGPRINTLN2("go to scale calibration");
            oled_print(OB16, SETTING, "CALIBRATION", "", "");
            while ( digitalRead(VALID_BTN) ) { // wait VALID_BTN to be released
              delay(100);
            } 
            sensor_nb = get_sensor_nb(sensor_nb);
            initScale(sensor_nb,OLED_ON);
            calibrateScale(sensor_nb);
            go_to_sleep (sleep_time, OLED_ON);
          }
          if (digitalRead(NEXT_BTN) ){
            create_ble_uart(ds.macSTA);   
            i = ble_settings_choice();
          }
          delay(10);
        } while (i == 0);
        switch ( i ) {              
          case 1 : DEBUGPRINTLN2("go to general settings");
                   oled_print(OB24, SETTING, "GENERAL", "", "");
                   ble_settings_general();
                   break;
          case 2 : DEBUGPRINTLN2("go to Lora settings");
                   oled_print(OB24, SETTING, "LORA", "", "");
                   ble_settings_lora(LoRa_param_1);
                   break;
          case 3 : DEBUGPRINTLN2("go to temperature drift");
                   oled_print(OC24, SETTING, TEMP_DRIFT1, TEMP_DRIFT2, "");
                   ble_settings_temperature_drift();
                   break;
          case 4 : DEBUGPRINTLN2("INIT EEPROM");
                   init_eeprom();
                   #ifdef T_18B20
                   init_temp_sensor();
                   #endif
                   oled_print(OB24, INIT_SYS1, INIT_SYS2, INIT_SYS3, "");
                   delay(AFF_TIME);
                   break;
          default: DEBUGPRINTLN2("bad choice");
                   break;
        }
        sleep_time = sleep_time_values[sleep_time_day_id];
        go_to_sleep (sleep_time, OLED_ON);
      }
    delay(10);
    }
    // check if the scale has an id
    if ( !scale_id )  {
      digitalWrite(LED_R, HIGH); digitalWrite(LED_G, HIGH);
      DEBUGPRINTLN3("No scale id, use ble setting to set it");
      oled_print(OC16, NO_SCALE_ID1, NO_SCALE_ID2, NO_SCALE_ID3 , NO_SCALE_ID4);
      digitalWrite(LED_R, LOW); digitalWrite(LED_G, LOW);
      previousTime = millis();
      while ( millis() < WAIT_TIME_FOR_CONFIG + previousTime ) {
        if ( digitalRead(NEXT_BTN) ) {
          DEBUGPRINTLN2("go to general settings");
          oled_print(OB24, SETTING, "GENERAL", "", "");
          create_ble_uart(ds.macSTA);   
          ble_settings_general();
        }
          
      }
    }
    DEBUGPRINT2("Select action:");
    ds.action = get_action();
    DEBUGPRINT2(ds.action); DEBUGPRINT2("="); DEBUGPRINT2(action_text[ds.action][0]);
    DEBUGPRINT2(" "); DEBUGPRINTLN2(action_text[ds.action][1]);
    switch ( ds.action ) { //0:no_action,1:Sirup50/50,2:Sirup70/30,3:Candy,4:materiel,5:Visit,6:measure only,7:sleep,8:infos bat,9:version,10:lora,11:temp drift
      case 0 : // no_action
        oled_print(OB24, NO_ACTION1, NO_ACTION2, NO_ACTION3, "");
        delay(AFF_TIME);
        go_to_sleep (sleep_time, OLED_ON);
      case 7 : // sleep
        go_to_sleep (sleep_time_values[sleep_time_for_visit_id], OLED_ON);
      case 8 : // infos bat
        oled_print(OB24, INFO_BAT, String(ds.v_bat) + VOLT, (ds.charg_bat != 255 ? String(ds.charg_bat) : "---") + PERCENT, "");
        delay(AFF_TIME);
        go_to_sleep (sleep_time, OLED_ON);
      case 9 : // version
        oled_print(OC24, "N° " + (String)scale_id, "Ver. " + (String)version , SAMPLE_NB + (String)SAMPLES + "x" + (String)READINGS, "");
        delay(AFF_TIME);
        go_to_sleep (sleep_time, OLED_ON);
      case 10: // lora
        oled_print(OB16, "LoRa",
            "f:" + String(frequency_values[LoRa_param_1.frequency_id]/1000000.0) + "MHz", 
            "sf:" + String(LoRa_param_1.spreadingFactor) + " - P=" + String(LoRa_param_1.txPower) + "dB",
            "bw:" + String(signalBandwidth_values[LoRa_param_1.signalBandwidth_id]/1000.0) + "kHz");
        delay(AFF_TIME);
        go_to_sleep (sleep_time, OLED_ON);
      case 11: // temp drift
        #ifdef T_18B20
          if ( temp_flag ) {
            while ( millis() < requestTime + 1000 ); // 750 ms : convertion time @ resolution 12 bits
            DEBUGPRINT0("TIME get temperature : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
            ds.temp = get_temp();
            temp_sensor.depower();
            ds.hygro = 255;
            if (ds.temp < -50.0 || ds.temp > 80.0  ) {
              //error_flags |= 0b00100000;
              DEBUGPRINTLN3("temp measure error");
            }
            DEBUGPRINT0("temperature = "); DEBUGPRINT0(ds.temp); DEBUGPRINTLN0("°C");
            DEBUGPRINT3("hygrometry  = "); DEBUGPRINT3(ds.hygro); DEBUGPRINTLN3("%");
          }
        #else
          ds.temp = 155;
          //error_flags |= 0b00100000;
          ds.hygro = 255;
        #endif
        oled_print(OC16, (String)temp_factor + " g/°C" , TEMP_DRIFT1, TEMP_DRIFT2, (String)ds.temp + " °C");
        delay(AFF_TIME);
        go_to_sleep (sleep_time, OLED_ON);   
      case 12: // OTA
        OTAWebUpdate();  
        go_to_sleep (sleep_time, OLED_ON);
      default: oled_print(OB24, MEASURE1, MEASURE2, MEASURE3, "");
    }
  }

  // blinking green led during measures
  tickerSetLow.attach_ms(BLINK_TIME_ON, setPinLow, LED_G);
  tickerSetHigh.attach_ms(BLINK_TIME_OFF, setPinHigh, LED_G);


  

  // ----------------- scale ------------
  // first get/set nb of sensors
  if (!scale_calibration_flag) {
    sensor_nb = get_sensor_nb(sensor_nb);
  }
  DEBUGPRINT3(sensor_nb);  DEBUGPRINTLN3(" sensor(s)");
  
  // init scale : gain, voltage, sample rate
  initScale(sensor_nb, ds.action);  
  
  // scale calibration
  if (!scale_calibration_flag) {
    calibrateScale(sensor_nb);
  }

  // get weight
  DEBUGPRINT0("TIME Sensor 0 : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
  i = 0 ; error_flags = 0;
  while ( get_1sigma_Average(SENSOR_0, &onScale, SAMPLES, READINGS) && i < get_ave_max_tries ) {  
    DEBUGPRINTLN2("error sensor A");
    error_flags |= 0b00010000;
    i++;
  }
  ds.mass = ((float)onScale-(float)zeroOffset[0])/calibrationFactor[0];
  DEBUGPRINT2(" ** mass 1: "); DEBUGPRINTLN2(((float)onScale-(float)zeroOffset[0])/calibrationFactor[0]);

  if ( sensor_nb > 1 ) {
    DEBUGPRINT0("TIME Sensor 1 : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
    i = 0 ;
    while ( get_1sigma_Average(SENSOR_1, &onScale, SAMPLES, READINGS) && i < get_ave_max_tries ) {  
      DEBUGPRINTLN2("error sensor B");
      error_flags |= 0b00010000;
      i++;
    }
    ds.mass += ((float)onScale-(float)zeroOffset[1])/calibrationFactor[1];
    DEBUGPRINT2(" ** mass 2: "); DEBUGPRINT2(((float)onScale-(float)zeroOffset[1])/calibrationFactor[1]);
    DEBUGPRINT2(" mass t: "); DEBUGPRINTLN2(ds.mass);
  }
  scale0.powerDown();

  if ( sensor_nb == 4 ) {
    scale1.powerUp(); delay(100);
    DEBUGPRINT0("TIME Sensor 2 : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
    i = 0 ;
    while ( get_1sigma_Average(SENSOR_2, &onScale, SAMPLES, READINGS) && i < get_ave_max_tries ) {  
      DEBUGPRINTLN2("error sensor C");
      error_flags |= 0b00010000;
      i++;
    }
    ds.mass += ((float)onScale-(float)zeroOffset[2])/calibrationFactor[2];
    DEBUGPRINT2(" ** mass 3: "); DEBUGPRINT2(((float)onScale-(float)zeroOffset[2])/calibrationFactor[2]);
    DEBUGPRINT2(" mass t: "); DEBUGPRINTLN2(ds.mass);
    DEBUGPRINT0("TIME Sensor 3 : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
    i = 0 ;
    while ( get_1sigma_Average(SENSOR_3, &onScale, SAMPLES, READINGS) && i < get_ave_max_tries ) {  
      DEBUGPRINTLN2("error sensor D");
      error_flags |= 0b00010000;
      i++;
    }
    ds.mass += ((float)onScale-(float)zeroOffset[3])/calibrationFactor[3];
    DEBUGPRINT2(" ** mass 4: "); DEBUGPRINT2(((float)onScale-(float)zeroOffset[3])/calibrationFactor[3]);
    DEBUGPRINT2(" mass t: "); DEBUGPRINTLN2(ds.mass);
    scale1.powerDown();
  }
  Serial.print("TIME end mass measures : "); Serial.print(millis()); Serial.println(" ms ***************");

  // temperature
#ifdef T_18B20
  if ( temp_flag ) {
    while ( millis() < requestTime + 1000 ); // 750 ms : convertion time @ resolution 12 bits
    DEBUGPRINT0("TIME get temperature : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
    ds.temp = get_temp();
    while ( (ds.temp < -50.0 || ds.temp > 80.0) && ( read_temp_tries < MAX_READ_TEMP_TRIES ) ) { // error re-read
      temp_sensor.reset();
      temp_sensor.select(temp_addr);
      temp_sensor.write(0x44, 0);        // start conversion
      DEBUGPRINT0("TIME request temperature ("); DEBUGPRINT0(read_temp_tries); DEBUGPRINT0(") : "); 
      DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
      requestTime = millis();
      while ( millis() < requestTime + 1000 ); // 750 ms : convertion time @ resolution 12 bits
      DEBUGPRINT0("TIME get temperature : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");
      ds.temp = get_temp();
      read_temp_tries++;
    }
    temp_sensor.depower();
    ds.hygro = 255;
    if ( ds.temp < -50.0 || ds.temp > 80.0  ) {
      error_flags |= 0b00100000;
      DEBUGPRINTLN3("temp measure error");
    }
    DEBUGPRINT0("temperature = "); DEBUGPRINT0(ds.temp); DEBUGPRINTLN0("°C");
    DEBUGPRINT3("hygrometry  = "); DEBUGPRINT3(ds.hygro); DEBUGPRINTLN3("%");
  }
#else
  ds.temp = 155;
  error_flags |= 0b00100000;
  ds.hygro = 255;
#endif
  
  ds.mass -= massOffset;
  DEBUGPRINT2("mass offset: "); DEBUGPRINTLN2(massOffset);
  DEBUGPRINT0("** mass total: "); DEBUGPRINTLN0(ds.mass);
  if ( temp_factor != 0 ) {
    ds.mass -= temp_factor * ( ds.temp - calibration_temp );
  }
  DEBUGPRINT0("mass corrected : "); DEBUGPRINT0(ds.mass);
  DEBUGPRINT0(" (calibration temperature : "); DEBUGPRINT0(calibration_temp);
  DEBUGPRINT0("°C / delta "); DEBUGPRINT0(ds.temp - calibration_temp); DEBUGPRINT0("°C / correction "); 
  DEBUGPRINT0( -temp_factor * ( ds.temp - calibration_temp )); DEBUGPRINTLN0("g)"); 
  sprintf(buf_mass,"%5.2f kg",ds.mass/1000);
  DEBUGPRINT0("** mass : "); DEBUGPRINT0(buf_mass); DEBUGPRINT0(" - error : "); DEBUGPRINTLN0(error_flags);
  if ( ds.mass < 0 ) {
    ds.mass = 0;
    error_flags |= 0b00001000;
    DEBUGPRINT0("mass < 0 , corrected to 0 "); DEBUGPRINT0(" - error : "); DEBUGPRINTLN0(error_flags);
  }
  DEBUGPRINT0("TIME End Sensors : "); DEBUGPRINT0(millis()); DEBUGPRINTLN0(" ms ***************");

  if (ds.action)
    oled_print(OB24, String(buf_mass), "", String(ds.temp) + " °C", "");

  // delta mass alarm
  if ( abs(ds.mass - previous_mass) > (float)delta_mass ) {
    error_flags |= 0b00100000;
    DEBUGPRINT0("delta mass exceeded "); DEBUGPRINT0(" - error : "); DEBUGPRINTLN0(error_flags);
  }
  previous_mass = ds.mass;
  
  // End measures
  Serial.print("TIME End measures : "); Serial.print(millis()); Serial.println(" ms ***************");

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
