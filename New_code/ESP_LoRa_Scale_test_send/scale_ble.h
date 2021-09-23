#ifndef scale_ble_h
#define scale_ble_h

/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/

enum lf {LINE_FEED_OFF, LINE_FEED_ON};
#define WAIT_TIME_FOR_BLE  60000/FREQ_COEF  // milli seconds
#define PARAM_LENGTH 50 // prévoir 3 de plus pour CR NL ....
char answer[PARAM_LENGTH];
char ble_buf[PARAM_LENGTH];
int val=0;


// #define BLE_SERVER "Lora Gateway" -> in langage.h

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
BLECharacteristic * pRxCharacteristic;
//std::string rxValue;
bool deviceConnected = false;
//bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

void create_ble_uart ();
void write_to_ble_terminal (char * text, bool line_feed);
void read_ble_terminal (char * read_value, uint8_t len, char * write_value);
void set_parameters();

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      DEBUGPRINTLN3("onConnect => connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      DEBUGPRINTLN3("onDisconnect => disconnected");
    }
};

//class MyCallbacks: public BLECharacteristicCallbacks {
//    void onWrite(BLECharacteristic *pCharacteristic) {
//      std::string rxValue = pCharacteristic->getValue();
//
//      if (rxValue.length() > 0) {
//        Serial.println("*********");
//        Serial.print("Received Value: ");
//        for (int i = 0; i < rxValue.length(); i++)
//          Serial.print(rxValue[i]);
//
//        Serial.println();
//        Serial.println("*********");
//      }
//    }
//};




#endif /* scale_ble_h */
