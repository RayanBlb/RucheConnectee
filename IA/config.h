/*  Config RuchESIEA
    M. Fournier G. Heiss
    2021, ESIEA
*/

/** Modules **/
bool FTP_UPLOAD = false;
bool ONE_WIRE = false;
bool AI = true;
bool DHT22 = false;
bool SCALE = false;
bool OTA = false;

/** Config **/
/* Infos */
char* BOARD = "WEMOS-LOLIN32-LITE";
char* SERIAL_NUMBER = "RuchESIEA";
#define FW_VERSION 5
/* Pins */
#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14
#define I2S_PORT I2S_NUM_0
#define LOADCELL_DOUT_PIN 16
#define LOADCELL_SCK_PIN 5
#define DHT22_PIN 23
#define ONE_WIRE_PIN 4
/* Audio */
#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024)
#define RECORD_TIME       (30) //Seconds
#define I2S_CHANNEL_NUM   (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)
/* Preset */
#define TARE_ON_BOOT 0
#define ERASE_CONFIG_ON_BOOT 0
#define ERASE_FEATURE_ON_BOOT 0
/* Wi-Fi */
#define TIMEOUT 5000
char* WIFI_SSID = "Mon point d'accès Wi-Fi";
char* WIFI_LOGIN = "";
char* WIFI_PASSWORD = "Mon mot de passe Wi-Fi";
char* FTP_SERVER = "IP du serveur FTP";
char* FTP_USER = "Nom d'utilisateur FTP";
char* FTP_PASSWORD = "Mot de passe FTP";
char* HTTP_SERVER = "https://ruch.esiea.fr";
char* API_KEY = "Clé API à copier depuis la page 'Ma Ruche'";
/* NTP */
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;
/* Misc */
float calibration_factor = 26000.f;
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP 3600
#define SCALE_READ 20
/* One Wire */
int NOMBRE_ONE_WIRE = 9; // Multiple de 9
int NOMBRE_ONE_WIRE_PAR_CADRE = 9;  // Ne pas toucher
const short ORDRE_ONE_WIRE[] = {3, 4, 2, 5, 1, 7, 6, 8, 9};

/* Global variables */
float weight = 0.0;
float tare_value = 0.0;
float temperature = 0.0;
float humidity = 0.0;
float battery_percent = 0.0;
String ai_state = "unkown";
float* one_wire_values;
