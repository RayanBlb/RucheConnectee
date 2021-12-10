/*  Update RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/

#include "Update.h"

void check_update(bool personnal_update) {
  Serial.println("Checking firmware update");
  HTTPClient http;
  http.setTimeout(TIMEOUT);

  String serverPath = HTTP_SERVER;
  serverPath.concat("/last_version");

  // Your Domain name with URL path or IP address with path
  http.begin(serverPath.c_str());

  // Send HTTP GET request
  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) {
    Serial.println(httpResponseCode);
    String payload = http.getString();
    int available_version = payload.toInt();
    if (available_version > FW_VERSION || personnal_update) {
      Serial.println("Firmware update available");
      serverPath = HTTP_SERVER;
      serverPath.concat("/last_firmware.bin");
      if (personnal_update) {
        Serial.println("Personnal update");
        serverPath.concat("?api_key=");
        serverPath.concat(API_KEY);
      }
      http.begin(serverPath.c_str());
      int httpResponseCode = http.GET();
      if (httpResponseCode == 200) {
        Serial.println("Downloading firmware...");
        File firmware = LITTLEFS.open("/firmware.bin", FILE_WRITE);
        int len = http.getSize();
        uint8_t buff[128] = { 0 };
        WiFiClient * stream = http.getStreamPtr();

        while (http.connected() && (len > 0 || len == -1)) {
          size_t size = stream->available();
          if (size) {
            int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
            firmware.write(buff, c);
            if (len > 0) {
              len -= c;
            }
          }
          delay(1);
        }
        firmware.close();
        Serial.println("Firmware downloaded");
        File new_firmware = LITTLEFS.open("/firmware.bin", FILE_READ);

        if (!new_firmware) {
          Serial.println("Failed to open file for reading");
          return;
        }

        Serial.println("Starting update...");


        size_t fileSize = new_firmware.size();

        if (!Update.begin(fileSize)) {

          Serial.println("Cannot do the update");
          Update.printError(Serial);
          return;
        };

        Update.writeStream(new_firmware);

        if (Update.end()) {

          Serial.println("Successful update");
        } else {

          Serial.println("Error Occurred: " + String(Update.getError()));
          return;
        }

        new_firmware.close();

        Serial.println("Reset in 3 seconds...");
        delay(3000);

        ESP.restart();

      } else {
        Serial.print("Can't download firmware: ");
        Serial.println(httpResponseCode);
      }
    } else {
      Serial.println("Already up-to-date");
    }
  } else {
    Serial.print("Can't get last firmware version: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();
}
