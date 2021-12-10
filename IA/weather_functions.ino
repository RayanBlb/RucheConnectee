/*  Weather RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/

#include <OneWire.h>
#include "DHT.h"
#define DHTTYPE DHT22
DHT dht(DHT22_PIN, DHTTYPE);
OneWire ds(ONE_WIRE_PIN);

void init_dht22() {
  dht.begin();
  Serial.println("DHT22 init done");
}

float read_dht22_temp() {
  temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Can't read DHT22");
    return -1;
  }
  Serial.print("[DHT22] Temperature : ");
  Serial.println(temperature);
  return temperature;
}

float read_dht22_hum() {
  humidity = dht.readHumidity();
  if (isnan(humidity)) {
    Serial.println("Can't read DHT22");
    return -1;
  }
  Serial.print("[DHT22] Humidity : ");
  Serial.println(humidity);
  return humidity;
}

float read_single_one_wire(short number) {
  byte data[9], addr[8];
  ds.reset_search();

  for (int j = 0; j < number; j++) {
    if (!ds.search(addr)) {
      // Pas de capteur
      Serial.print(j);
      Serial.println(" No sensor");
      //return NO_SENSOR_FOUND;
    }
  }

  Serial.print("Adresse : ");
  for (int i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX);
  }
  Serial.println("");

  if (OneWire::crc8(addr, 7) != addr[7]) {
    // Adresse invalide
    Serial.println("Invalid address");
    return -1;
  }

  if (addr[0] != 0x28) {
    // Mauvais type de capteur
    Serial.println("Invalid sensor");
    return -1;
  }

  ds.reset();
  ds.select(addr);

  ds.write(0x44, 1);
  delay(800);

  ds.reset();
  ds.select(addr);
  ds.write(0xBE);

  for (byte i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  float temperature = (int16_t) ((data[1] << 8) | data[0]) * 0.0625;


  Serial.print(F("Temperature : "));
  Serial.print(temperature, 2);
  Serial.write(176); // Caractère degré
  Serial.write('C');
  Serial.println();
  return temperature;

}

void read_all_one_wire() {
  int i;
  for (i = 0; i < NOMBRE_ONE_WIRE; i++) {
    Serial.print("Reading one wire number ");
    Serial.println(i);
    one_wire_values[i] = read_single_one_wire(ORDRE_ONE_WIRE[i]);
  }
}
