/*  File system RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/

#include <LITTLEFS.h>

void LITTLEFSInit() {
  if (!LITTLEFS.begin(true)) {
    Serial.println("LITTLEFS initialisation failed! Formatting...");
    LITTLEFS.format();
    Serial.println("Formatting done");
  } else {
    Serial.println("LITTLEFS OK");
  }
  LITTLEFS.remove("/firmware.bin");
  LITTLEFS.remove("/recording.wav");
}

void load_features() {
  File feature_file = LITTLEFS.open("/feature.txt", FILE_READ);
  if (!feature_file || ERASE_FEATURE_ON_BOOT) {
    Serial.println("Can't open feature file or ERASE_FEATURE_ON_BOOT, default feature will be written");
    feature_file = LITTLEFS.open("/feature.txt", FILE_WRITE);
    if (FTP_UPLOAD) {
      feature_file.println("FTP_UPLOAD");
    }
    if (ONE_WIRE) {
      feature_file.println("ONE_WIRE");
    }
    if (AI) {
      feature_file.println("AI");
    }
    if (DHT22) {
      feature_file.println("DHT22");
    }
    if (SCALE) {
      feature_file.println("SCALE");
    }
    if (OTA) {
      feature_file.println("OTA");
    }
  } else {
    String line;
    short line_count = 0;
    FTP_UPLOAD = AI = ONE_WIRE = DHT22 = SCALE = OTA = false;
    while (feature_file.available()) {
      line = feature_file.readStringUntil('\n');
      if (line.indexOf("FTP_UPLOAD") == 0) {
        FTP_UPLOAD = true;
      } else if (line.indexOf("AI") == 0) {
        AI = true;
      } else if (line.indexOf("ONE_WIRE") == 0) {
        ONE_WIRE = true;
      } else if (line.indexOf("DHT22") == 0) {
        DHT22 = true;
      } else if (line.indexOf("SCALE") == 0) {
        SCALE = true;
      } else if (line.indexOf("OTA") == 0) {
        OTA = true;
      } else {
        Serial.print("Unknown feature ");
        Serial.println(line);
      }
    }
  }
  feature_file.close();
  Serial.println("Features OK");
}

void load_config() {
  File config_file = LITTLEFS.open("/config.txt", FILE_READ);
  if (!config_file || ERASE_CONFIG_ON_BOOT) {
    Serial.println("Can't open config file or ERASE_CONFIG_ON_BOOT, default config will be written");
    config_file = LITTLEFS.open("/config.txt", FILE_WRITE);
    config_file.println(WIFI_SSID);
    config_file.println(WIFI_LOGIN);
    config_file.println(WIFI_PASSWORD);
    config_file.println(API_KEY);
    config_file.println(BOARD);
    config_file.println(SERIAL_NUMBER);
    Serial.println("Config written");
  } else {
    String line;
    short line_count = 0;
    while (config_file.available()) {
      line = config_file.readStringUntil('\n');
      switch (line_count) {
        case 0:
          WIFI_SSID = (char*)malloc(128);
          line.toCharArray(WIFI_SSID, (int)line.length());
          break;
        case 1:
          WIFI_LOGIN = (char*)malloc(128);
          line.toCharArray(WIFI_LOGIN, (int)line.length());
          break;
        case 2:
          WIFI_PASSWORD = (char*)malloc(128);
          line.toCharArray(WIFI_PASSWORD, (int)line.length());
          break;
        case 3:
          API_KEY = (char*)malloc(128);
          line.toCharArray(API_KEY, (int)line.length());
          break;
        case 4:
          BOARD = (char*)malloc(128);
          line.toCharArray(BOARD, (int)line.length());
          break;
        case 5:
          SERIAL_NUMBER = (char*)malloc(128);
          line.toCharArray(SERIAL_NUMBER, (int)line.length());
          break;
        default:
          break;
      }
      line_count++;
    }
  }
  config_file.close();
  Serial.println("Config OK");
}
