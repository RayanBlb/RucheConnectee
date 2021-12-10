/*  Scale RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/

#include <HX711.h>
HX711 scale;

void ScaleInit() {
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.power_up();
  scale.set_scale();
  File tare_file = LITTLEFS.open("/tare.txt", FILE_READ);
  if (!tare_file || TARE_ON_BOOT) {
    Serial.println("Tare file doesn't exist or TARE_ON_BOOT, taring...");
    delay(400);
    if (scale.wait_ready_timeout(TIMEOUT)) {
      tare_value = scale.get_units(SCALE_READ) / calibration_factor;
    } else {
      Serial.println("Can't read scale, restarting...");
      ESP.restart();
    }
    File tare_file = LITTLEFS.open("/tare.txt", FILE_WRITE);
    tare_file.println(tare_value, 8);
    tare_file.close();
    Serial.println("Tare file written");
  } else {
    Serial.println("Tare file exists, reading...");
    unsigned char buf[1024];
    tare_file.read(buf, sizeof(buf));
    tare_value = atof((const char*)buf);
    tare_file.close();
    Serial.println("Tare file read");
  }
  Serial.print("Tare set to ");
  Serial.println(tare_value, 8);
  scale.power_down();
  Serial.println("Scale init done");
}

float read_scale() {
  Serial.println("Reading scale...");
  scale.power_up();
  delay(400);
  Serial.println("Scale powered up");
  float raw_val;
  if (scale.wait_ready_timeout(TIMEOUT)) {
    raw_val = scale.get_units(SCALE_READ);
  } else {
    Serial.println("Can't read scale");
    weight = -1;
    return -1;
  }
  Serial.print("Scale raw value: ");
  Serial.println(raw_val / calibration_factor);
  float val = -((raw_val / calibration_factor) - tare_value);
  Serial.println("Scale read");
  scale.power_down();
  Serial.println("Scale powered down");
  weight = val;
  Serial.print("Scale read: ");
  Serial.println(val, 10);
  return val;
}

void force_tare(String value) {
  Serial.println("New defined tare");
  File tare_file = LITTLEFS.open("/tare.txt", FILE_WRITE);
  tare_file.println(value);
  tare_file.close();
  tare_value = atof(value.c_str());
  Serial.print("New tare defined ");
  Serial.println(tare_value);
}

void force_tare() {
  Serial.println("New tare");
  if (scale.wait_ready_timeout(TIMEOUT)) {
    tare_value = scale.get_units(SCALE_READ) / calibration_factor;
  } else {
    Serial.println("Can't read scale");
    return;
  }
  File tare_file = LITTLEFS.open("/tare.txt", FILE_WRITE);
  tare_file.println(tare_value, 8);
  tare_file.close();
  Serial.println("New tare file written");
}
