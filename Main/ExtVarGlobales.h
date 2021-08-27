// Variable Main //

extern String LoraSendChaine;
extern int Counter;
//  Variable Son //


extern const uint16_t samples; //This value MUST ALWAYS be a power of 2
extern const double samplingFrequency; //Hz, must be less than 10000 due to ADC

extern unsigned int sampling_period_us;
extern unsigned long microseconds;
       
extern double vReal[];
extern double vImag[];

//  Variable Piezo //

extern const uint16_t samples_piezo; //This value MUST ALWAYS be a power of 2
extern const double samplingFrequency_piezo; //Hz, must be less than 10000 due to ADC

extern unsigned int sampling_period_us_piezo;
extern unsigned long microseconds_piezo;
       
extern double vReal_piezo[];
extern double vImag_piezo[];

//  Variable CO2 //

extern String ChaineCO2;
extern float temp;

/**************************************/
//  Variable Lora                     //
/**************************************/

extern String LoRaData;
