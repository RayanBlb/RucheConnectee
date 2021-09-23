/* weight scale for bee hives */
/* see : http://rucher.polytech.unice.fr/index.php
 *       https://github.com/christian-peter/ruche-connecte/
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

#ifndef scale_language_h
#define scale_language_h

#if ( LANGUAGE == 'F') 
 #define INIT_MSG1 "Balance"
 #define INIT_MSG2 "ruche"

 #define INIT_SYS1 "RESET"
 #define INIT_SYS2 "GLOBAL"
 #define INIT_SYS3 "(EEPROM)"

 #define ACTION_SELECT1 "SELECT."
 #define ACTION_SELECT2 "ACTION"
 const char *action_text[13][2] = {"AUCUNE","ACTION","NOURRISS."," 50/50","NOURRISS."," 70/30","NOURRISS.",
                                  " CANDY","MODIF.","MATERIEL","CREATION","VISITE","MESURE ","MASSE",
                                  "MISE EN","SOMMEIL","INFO","BATTERIE","INFO","VERSION","INFO","LoRa","INFO","TEMP.",
                                  "Transfert","sketch-ota"};
 #define NO_ACTION1 "Pas"
 #define NO_ACTION2 "d'action"
 #define NO_ACTION3 "définie"
 
 #define MEASURE1 "MESURES"
 #define MEASURE2 "EN COURS"
 #define MEASURE3 "PATIENCE"

 #define INFO_BAT "BATTERIE"
 #define VOLT     " V"
 #define PERCENT  " %"

 #define NO_ACK1 "DONNEES"
 #define NO_ACK2 "NON"
 #define NO_ACK3 "RECUES"

 #define SEND "ENVOI N°"

 #define NB_OF_SENSORS1 "Nb de capteurs"
 #define NB_OF_SENSORS2 "1, 2 ou 4"

 #define CHOOSE "choisir avec (->)"
 #define VALID  "(<-) puis validez"

 #define CALIBRATION1 "Mode étalonnage"
 #define CALIBRATION2 "étalonnage"
 #define CALIBRATION3 "Videz le plateau"
 #define CALIBRATION3A "Retirez plateau"
 #define CALIBRATION3B "Reposer plateau"
 #define CALIBRATION4 "puis pressez (->)"

 #define ZERO1 "Mode"
 #define ZERO2 "étalonnage"
 #define ZERO3 "calcul tare "
 #define ZERO4 "patientez"

 #define TEMP_FACTOR1 "Dérive température"
 
 #define MASS1 "Masse d'étalonnage"
 #define MASS2 ""
 #define MASS3 "    choisir avec (->)"
 #define MASS4 "(<-) suivant/validez"

 #define CAL1 "Placez la masse"
 #define CAL2 "   sur la jauge"
 #define CAL3 ""
 #define CAL4 "puis pressez (->)"

 #define CAL5 "Mode"
 #define CAL6 "calibration"
 #define CAL7 "jauge "
 #define CAL8 "patientez"

 

 #define BLE_SERVER  "Ruche"
 
 #define BLE_INVIT1 "ne pas relacher"
 #define BLE_INVIT2 "pour entrer"
 #define BLE_INVIT3 "en mode"
 #define SETTING "REGLAGES"
 #define SAVED   "ENREGISTRES"






 #define GENERAL  "Général"
 #define LORA     "LoRa"
 #define CALIBRATION "Calibration"
 #define TEMP_DRIFT  "Dérive en température"
 #define TEMP_DRIFT1  "Dérive en"
 #define TEMP_DRIFT2  "température"
 #define RST_EEPROM "Init EEPROM"
 #define CHOICE    "choix:"

 #define STRING_TO_LONG "trop long"

 #define BLE_CONNECT1 "Connection au"
 #define BLE_CONNECT2 "serveur bluetooth"
 #define BLE_CONNECT3 "attendue"
 #define BLE_CONNECT4 "établie"

 #define TEMP_MEASURE1  "Mesure de la"
 #define TEMP_MEASURE2  "température"


 #define TEMP_ERROR1  "Pas de mesure"
 #define TEMP_ERROR2  "de température"
 #define TEMP_ERROR3  "cablâge ou "
 #define TEMP_ERROR4  "reset EEPROM"

 #define SAMPLE_NB "Echant. "

 #define WIFI     "WiFi"



 
 
 #define LR_TXPOWER "Puissance" //(dB)"
 #define LR_FREQ    "Fréquence" //(Mhz)"
 #define LR_SF      "SF (portée)"
 #define LR_SBW     "SBW" // (kHz)"

 


 #define MODE "Mode ...."
 #define CHOICES "Gén. LoRa Cal."
 const char *choice[] = {"LoRa","Calibration","Général"};


 #define SLEEP_TIME       "Interval mesure" 
 #define SLEEP_TIME_DAY   "jour "
 #define SLEEP_TIME_NIGHT "nuit "
 #define SLEEP_TIME_VISIT "visite "
 #define SEND_MAX_TRIES   "Transmissions"
 #define SEND_MAX_TRIES1  "nb maxi "
 #define SEND_MAX_TRIES_NB  "Nb maxi de transmissions"

 #define SCALE_INDEX "Numéro (index) de la balance"

 #define DELTA_M "Variation de masse generant une \nalarme "

 #define LOGOUT "Déconnexion"

 #define GO_TO_SLEEP "Mise en sommeil pour "
 #define GO_TO_SLEEP1 "Mise en"
 #define GO_TO_SLEEP2 "sommeil"
 #define MN " mn"
 #define MINUTES " minutes"
 
 #define TEMP_DRIFT1 "Compensation"
 #define TEMP_DRIFT2 "en temperature"

 #define  NO_SCALE_ID1 "Pas de N° de ruche"
 #define  NO_SCALE_ID2 "Utilisez réglages "
 #define  NO_SCALE_ID3 "bluetooth (Général)"
 #define  NO_SCALE_ID4 "pour le saisir  -->"
  
 
#else /* LANGAGE */
 // the same in GB

#endif /* LANGAGE */



#endif /* scale_language_h */
