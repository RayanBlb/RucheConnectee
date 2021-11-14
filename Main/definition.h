/**************************************/
//  Variable Lora                     //
/**************************************/

//define the pins used by the LoRa transceiver module
#define SCK 18
#define MISO 19
#define MOSI 23
#define CS 18

#define SS 5
#define RST 14
#define DI0 26

/**************************************/
//  Variable Fonction                 //
/**************************************/

//Viable defini
#define MEASURE 13
#define uS_TO_S_FACTOR 1000000  /* Conversion seconde to micro seconde */
#define TIME_TO_SLEEP  3600       /* Temps Sleep (en seconde)*/

//  Variable Son 
#define CHANNEL_SON 4

//  Variable Piezo 
#define CHANNEL_PIEZO 2

//Taille payload
#define PAYLOAD_LEN 16
