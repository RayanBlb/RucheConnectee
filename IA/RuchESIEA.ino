/*  Main RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/

#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "config.h"
#include <ESP32_FTPClient.h>

RTC_DATA_ATTR int bootCount = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("\n*** RuchESIEA ***");
  Serial.print("Running firmware v");
  Serial.println(FW_VERSION);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(UINT64_C(TIME_TO_SLEEP * uS_TO_S_FACTOR));
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " seconds");
  LITTLEFSInit();
  load_config();
  load_features();
  Serial.print("I am ");
  Serial.println(SERIAL_NUMBER);
  if (SCALE) {
    ScaleInit();
  }
  recording_file_init();
  i2sInit();
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, NULL);
  delay(RECORD_TIME * 1000 + 2500);
  if (AI) {
    ai();
  }
  if (DHT22) {
    init_dht22();
    read_dht22_temp();
    delay(300);
    read_dht22_hum();
  }
  if (SCALE) {
    read_scale();
  }
  if (ONE_WIRE) {
    one_wire_values = (float*)malloc(sizeof(float) * NOMBRE_ONE_WIRE);
    read_all_one_wire();
  }
  WIFIInit();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.print("Today is ");
  Serial.println(get_datetime());
  get_remote_command();
  if (FTP_UPLOAD) {
    upload_to_ftp();
  }
  send_hwinfo();
  send_state();
  if (ONE_WIRE) {
    send_frames();
  }
  if (OTA) {
    check_update(false);
  }
  WiFi.disconnect();
  Serial.println("WiFi disconnected, going to sleep...");
  Serial.flush();
  delay(100);
  esp_deep_sleep_start();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}
