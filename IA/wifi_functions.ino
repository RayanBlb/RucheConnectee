/*  WiFI RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/

#include <WiFi.h>
#include "esp_wpa2.h"

void WIFIInit() {
  WiFi.disconnect();
  // If 802.1x mode
  if (strlen(WIFI_LOGIN) > 1) {
    Serial.println("Wifi 802.1x mode");
    WiFi.mode(WIFI_STA);
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)WIFI_LOGIN, strlen(WIFI_LOGIN));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)WIFI_PASSWORD, strlen(WIFI_PASSWORD));
    esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT(); //set config settings to default
    esp_wifi_sta_wpa2_ent_enable(&config); //set config settings to enable function
    WiFi.begin(WIFI_SSID); //connect to wifi
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  short wifi_count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    wifi_count++;
    if (wifi_count > 50) {
      Serial.println("Can't connect to WiFi, disconnecting and resetting...");
      WiFi.disconnect();
      ESP.restart();
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

char* get_datetime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "unknown";
  }
  char * ret = (char*)malloc(128);
  *ret = '\0';
  sprintf(ret, "%d-%d-%d_%dh%d", timeinfo.tm_mday, timeinfo.tm_mon + 1, 1900 + timeinfo.tm_year, timeinfo.tm_hour, timeinfo.tm_min);
  return ret;
}
