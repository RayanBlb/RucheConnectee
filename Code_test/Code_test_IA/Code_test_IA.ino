#include <LITTLEFS.h>
#include <driver/i2s.h>

#include "audio.h"
#include "ia.h"


void LITTLEFSInit() {
  if (!LITTLEFS.begin(true)) {
    Serial.println("LITTLEFS initialisation failed! Formatting...");
    LITTLEFS.format();
    Serial.println("Formatting done");
  } else {
    Serial.println("LITTLEFS OK");
  }
  LITTLEFS.remove("/recording.wav");
}

void setup(){

  Serial.begin(9600);
  Serial.println("\n*** RuchESIEA ***");
  LITTLEFSInit();
}

void loop() {
  recording_file_init();
  i2sInit();
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, NULL);
  delay(RECORD_TIME * 1000 + 2500);
  ai();
}
