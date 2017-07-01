/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai Würtz

Private-use only! (you need to ask for a commercial-use)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Private-use only! (you need to ask for a commercial-use)
*/

#include "stm32f4xx_hal.h"

#ifndef APPLICATION_USER_CALCULATION_H_
#define APPLICATION_USER_CALCULATION_H_

#define SENDER_ARRAY_SIZE  (480) //480 328 //240 // 92 96  //Bosch=328 pseudonoise4_dbpsk=240  Ardumower = 96
#define EMPF_ARRAY_SIZE   (SENDER_ARRAY_SIZE*3) //720 //288  //=> Signal wird mindestens 2x im Abtastintervall erkannt
#define CORELLATION_ARRAY_SIZE  (EMPF_ARRAY_SIZE - SENDER_ARRAY_SIZE + 1)
#define QUALITYTHRESHOLD   (1.49f) //Bosch=1.2 pseudonoise4_dbpsk = 1.5

typedef enum {
	false = 0, true = 1
} bool;

typedef struct {

	int16_t * empfangssignal;

	int32_t adcOffset;

	//Ergebnis per Aufruf von convolve()
	int32_t magnitude;         // Amplitude des Korrelationssignals
	float filterQuality; // Verhältnis maxAmplitude zu minAmplitude immer Positive
	int16_t peakDistanz;    // Distance between the two maxima

	// Wenn Spule andersherum angeschlossen ist
	//bool swapCoilPolarity = false;

	int32_t correlationsignal[CORELLATION_ARRAY_SIZE]; // Wird von convolve ausgefüllt. Wird benutzt um zweites maximum zu finden

} CAL_ChannelTypeDef;

extern void cal_init();
extern void cal_offset(CAL_ChannelTypeDef *conf);
extern void cal_convolve(CAL_ChannelTypeDef *conf);

extern void cal_printSignal(CAL_ChannelTypeDef *conf, int peakhigh, bool *flagShowSignal);
extern void cal_printADCSignal(CAL_ChannelTypeDef *conf, int peakhigh, bool *flagShowSignal);
extern void cal_printOffset(CAL_ChannelTypeDef *confL, CAL_ChannelTypeDef *confR, CAL_ChannelTypeDef *confB);

#endif /* APPLICATION_USER_CALCULATION_H_ */
