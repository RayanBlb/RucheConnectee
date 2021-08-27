/**************************************/
//  Variable Main                      //
/**************************************/

String SendLoraChaine;
int Counter;

/**************************************/
//  Variable Son                      //
/**************************************/

const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 2000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;
       
double vReal[samples];
double vImag[samples];

/**************************************/
//  Variable Piezo                    //
/**************************************/

const uint16_t samples_piezo = 1024; //This value MUST ALWAYS be a power of 2
const double samplingFrequency_piezo = 2000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us_piezo;
unsigned long microseconds_piezo;
       
 double vReal_piezo[samples_piezo];
double vImag_piezo[samples_piezo];

/**************************************/
//  Variable CO2                      //
/**************************************/
String ChaineCO2;
float temp;

/**************************************/
//  Variable Lora                     //
/**************************************/

String LoRaData;
