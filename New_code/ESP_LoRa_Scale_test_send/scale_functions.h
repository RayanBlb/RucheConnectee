/* weight scale for bee hives */
/* see : http://rucher.polytech.unice.fr/index.php
 * --------------------------------------------------
 * IMPORTANT : choose "Huge APP" in Tools -> Partition scheme
 * --------------------------------------------------
 * LICENCE
 * All rights reserved. 
 * This program and the accompanying materials are made available under the terms of the MIT License 
 * which accompanies this distribution, and is available at https://opensource.org/licenses/mit-license.php 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * --------------------------------------------------
 * Christian.PETER_at_univ-cotedazur.fr
 */

#ifndef scale_functions_h
#define scale_functions_h


#define EEPROM_SIZE 128
#define LOCATION_MAC_ADDRESS           0

#define LOCATION_LORA_TXPOWER          10  //  int  
#define LOCATION_LORA_FREQUENCY        14  //  long 
#define LOCATION_LORA_SPREADINGFACTOR  18  //  int  
#define LOCATION_LORA_SIGNALBANDWIDTH  22  //  long 
#define LOCATION_LORA_CODINGRATE       26  //  int  
#define LOCATION_LORA_PREAMBLE         30  //  long 
#define LOCATION_LORA_SYNCWORD         34  //  int  
#define LOCATION_LORA_OCP              38  //  uint8_t 

#define LOCATION_CALIBRATION_TEMP      46
#define LOCATION_TEMP_FACTOR           50
#define LOCATION_SCALEA_ZERO           54
#define LOCATION_SCALEA_CAL_FACTOR     58
#define LOCATION_SCALEB_ZERO           62 
#define LOCATION_SCALEB_CAL_FACTOR     66 
#define LOCATION_SCALEC_ZERO           70
#define LOCATION_SCALEC_CAL_FACTOR     74
#define LOCATION_SCALED_ZERO           78
#define LOCATION_SCALED_CAL_FACTOR     82
#define LOCATION_KNOWN_WEIGHT          86
#define LOCATION_MASS_OFFSET           90
#define LOCATION_SENSOR_NB             94
     
#define LOCATION_SLEEP_TIME_DAY        100   //uint16_t
#define LOCATION_SLEEP_TIME_NIGHT      102   //uint16_t
#define LOCATION_SLEEP_TIME_FOR_VISIT  104   //uint16_t
#define LOCATION_SEND_MAX_TRIES        106   //uint8_t

#define LOCATION_18B20_ROM             108   // 8 bytes

#define LOCATION_SCALE_ID              120   //uint16_t
#define LOCATION_DELTA_MASS            123   //float

//float get_voltage(adc1_channel_t channel,adc_atten_t channel_atten, int measure_nb, int divider_coef);
//
//void read_eeprom();
//
//uint8_t get_action();
//
//uint8_t get_charg_bat();


#endif /* scale_functions_h */
