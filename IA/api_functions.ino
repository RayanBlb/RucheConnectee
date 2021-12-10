/*  API RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/

#include <LITTLEFS.h>

void send_hwinfo() {
  Serial.println("Sending hardware info to the server");
  HTTPClient http;
  http.setTimeout(TIMEOUT);
  String serverPath = HTTP_SERVER;
  char * var_buffer = (char*)malloc(256);
  *var_buffer = '\0';
  sprintf(var_buffer, "/send_hwinfo?api_key=%s&board=%s&serial_number=%s&fw_version=%d&battery_percent=%.2f", API_KEY, BOARD, SERIAL_NUMBER, FW_VERSION, battery_percent);
  serverPath.concat(var_buffer);
  http.begin(serverPath.c_str());
  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {
    Serial.println("Hardware info sent");
  } else {
    Serial.print("Error: ");
    Serial.println(httpResponseCode);
  }
}

void send_frames() {
  Serial.println("Sending frames info to the server");
  HTTPClient http;
  http.setTimeout(TIMEOUT);
  String serverPath = HTTP_SERVER;
  serverPath.concat("/send_frame?api_key=");
  serverPath.concat(API_KEY);
  serverPath.concat("&one_wire_per_frame=");
  serverPath.concat(NOMBRE_ONE_WIRE_PAR_CADRE);
  int i;
  for(i = 0; i < NOMBRE_ONE_WIRE; i++) {
    serverPath.concat("&sensor_");
    serverPath.concat(i);
    serverPath.concat("=");
    serverPath.concat(one_wire_values[i]);
  }
  http.begin(serverPath.c_str());
  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {
    Serial.println("Frames info sent");
  } else {
    Serial.print("Error: ");
    Serial.println(httpResponseCode);
  }
}

void get_remote_command() {
  Serial.println("Getting remote command from server");
  HTTPClient http;
  http.setTimeout(TIMEOUT);
  String serverPath = HTTP_SERVER;
  char * var_buffer = (char*)malloc(256);
  *var_buffer = '\0';
  sprintf(var_buffer, "/get_command?api_key=%s", API_KEY);
  serverPath.concat(var_buffer);
  http.begin(serverPath.c_str());
  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {
    String payload = http.getString();
    String command = payload.substring(0, 4);
    if (command == "reset") {
      Serial.println("Restart command");
      ESP.restart();
    } else if (command == "updat") {
      Serial.println("Update command");
      check_update(true);
    } else if (command == "confi") {
      File config_file = LITTLEFS.open("config.txt", FILE_WRITE);
      config_file.println(payload.substring(5));
      config_file.close();
      Serial.println("New config file installed");
    } else if (command == "tare0") {
      force_tare();
    } else if (command == "taren") {
      force_tare(payload.substring(5));
    }
    Serial.println("End of remote command");
  } else {
    Serial.print("Error: ");
    Serial.println(httpResponseCode);
  }
}

void send_state() {
  Serial.println("Sending state to the server");
  HTTPClient http;
  http.setTimeout(TIMEOUT);
  String serverPath = HTTP_SERVER;
  char * var_buffer = (char*)malloc(256);
  *var_buffer = '\0';
  sprintf(var_buffer, "/send_state?api_key=%s&temperature=%.3f&humidity=%.3f&ai_state=%s&weight=%.3f&tare=%f", API_KEY, temperature, humidity, ai_state, weight, tare_value);
  serverPath.concat(var_buffer);
  http.begin(serverPath.c_str());
  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {
    Serial.println("State sent");
  } else {
    Serial.print("Error: ");
    Serial.println(httpResponseCode);
  }
}
