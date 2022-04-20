/**************************************/
//  Variable Lora                     //
/**************************************/

//define the pins used by the LoRa transceiver module
#define SCK 18 //V
#define MISO 19 //V
#define MOSI 23 //V

#define SS 5 //V
#define RST 14 //V
#define DI0 26 //V

#define I2S_WS 12 //V
#define I2S_SD 2 //V
#define I2S_SCK 15 //V
#define I2S_PORT I2S_NUM_0

/**************************************/
//  Variable Fonction                 //
/**************************************/

//Viable defini
#define MEASURE 13

//  Variable Son 
#define CHANNEL_SON 4

//  Variable Piezo 
#define CHANNEL_PIEZO 2

//Taille payload
#define PAYLOAD_LEN 20

/**************************************/
//  Variable Audio IA                 //
/**************************************/

#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024)
#define RECORD_TIME       (30) //Seconds
#define I2S_CHANNEL_NUM   (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)

