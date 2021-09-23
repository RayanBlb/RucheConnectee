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

#ifndef esp_lora_h
#define esp_lora_h

struct LoRa_param_s {
  int  txPower = 15; // 2 to 17 dB limited to 15 because the UCA antenne = +2dB
  long frequency_id = 2; // 863.2 to 868.1 Mhz
  long frequency_offset = 0; 
  int  spreadingFactor = 12; // 7 to 12
  long signalBandwidth_id = 1; // 125, 250, 500 kHz tester 62.5
  int  codingRateDenominator = 8 ; // 5 to 8
  long preambleLength = 12; // 6 to 65535, 12 default
  int  syncWord = 0x12F2; // 1 to 8 bytes ; Note Sync Word values containing 0x00 byte(s) are forbidden
  //uint8_t OCP; // Over Current Protection control (mA)
} LoRa_param_1;

long frequency_values[] = {863.2E6, 863.5E6, 863.8E6, 864.1E6, 864.4E6, 864.7E6, 865.2E6, 865.5E6, 
                           865.8E6, 866.1E6, 866.4E6, 867.7E6, 867.0E6, 868.0E6, 868.1E6}; // Hz
long signalBandwidth_values[] = {62500, 125000, 250000, 500000}; // Hz

#endif /* esp_lora_h */



/*
 * Explicit Header Mode:
In Explicit Header Mode, the presence of the CRC at the end of the payload in selected only on the transmitter side through
the bit RxPayloadCrcOn in the register RegModemConfig1.
Version:2.0
27/123
www.hoperf.comRFM95W/96W/98W
On the receiver side, the bit RxPayloadCrcOn in the register RegModemConfig1 is not used and once the payload has
been received, the user should check the bit CrcOnPayload in the register RegHopChannel. If the bit CrcOnPayload is at
‘1’, the user should then check the Irq Flag PayloadCrcError to make sure the CRC is valid.
If the bit CrcOnPayload is at ‘0’, it means there was no CRC on the payload and thus the IRQ Flag PayloadCrcError will not
be trigged even if the payload has errors.
 */

 /* Here is a reminder of the pre-defined radio frequency channels in the library:
(most of them from initial Libelium SX1272.h, except those marked with *)
ch  F(MHz)  ch  F(MHz)  ch  F(MHz)
04  863.2*  00  903.08  00  433.3*
05  863.5*  01  905.24  01  433.6*
06  863.8*  02  907.40  02  433.9*
07  864.1*  03  909.56  03  434.3*
08  864.4*  04  911.72  -   -
09  864.7*  05  913.88  -   -
10  865.2   06  916.04  -   -
11  865.5   07  918.20  -   -
12  865.8   08  920.36  -   -
13  866.1   09  922.52  -   -
14  866.4   10  924.68  -   -
15  867.7   11  926.84  -   -
16  867.0   12  915.00  -   -
17  868.0   -   -   -   -
18  868.1*  -   -   -   -

If you select ETSI_EUROPE_REGULATION then available channels are channel 
CH_10_868 to CH_18_868 (and the default channel will be CH_10_868 C. Pham).

There are also regulation concerning the maximum transmit power which
14dBm (25mW) for ETSI.*/
 
