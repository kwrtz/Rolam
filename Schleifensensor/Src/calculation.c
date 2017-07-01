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

#include "calculation.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

const int8_t sendersignal[SENDER_ARRAY_SIZE] = { //pseudonoise4_dbpsk #define SENDER_ARRAY_SIZE   480    Quality 1.5
	  	  -1,      -1, -1, 0,					0, 0, 0, 0,
		  1, 1,     1, 0,     0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1,      -1, -1, 0,					0, 0, 0, 0,
		  1,        1, 1, 0,					0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1,        1, 1, 0,					0, 0, 0, 0,
		  -1,      -1, -1, 0,					0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1,      -1, -1, 0,					0, 0, 0, 0,
		  1,        1, 1, 0,					0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1,        1, 1, 0,					0, 0, 0, 0,
		  -1,      -1, -1, 0,					0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1,      -1, -1, 0,					0, 0, 0, 0,
		  1,        1, 1, 0,					0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1,        1, 1, 0,					0, 0, 0, 0,
		  -1,      -1, -1, 0,					0, 0, 0, 0,
		  1, 1,     1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  -1,      -1, -1, 0,					0, 0, 0, 0,
		  1,        1, 1, 0,					0, 0, 0, 0,
		  -1, -1,  -1, 0,      0, 0, 0, 0,		0, 0, 0, 0,0, 0, 0, 0,
		  1,        1, 1, 0,					0, 0, 0, 0
};

extern UART_HandleTypeDef huart2;

// Maximale summe die eine correlationsberechung ergeben kann.
static int32_t maxCorelSumValue;

void cal_init() {

	// compute sum of absolute filter coeffs
	maxCorelSumValue = 0;
	for (int16_t i = 0; i < SENDER_ARRAY_SIZE; i++)
		maxCorelSumValue += abs(sendersignal[i]);

	// Calculate maximal possible value of one correlation magnitude
	maxCorelSumValue *= 4095;

}

void cal_offset(CAL_ChannelTypeDef *conf) {
	volatile int32_t mittelwert;

	// Mittelwert bzw. Offset berechnen
	mittelwert = 0;
	for (int16_t i = 0; i < EMPF_ARRAY_SIZE; i++) {
		mittelwert += conf->empfangssignal[i];
	}

	conf->adcOffset = mittelwert / (int32_t)EMPF_ARRAY_SIZE;

	/*
	if (conf->adcOffset > 2047)
		conf->adcOffset = 2047;
	else if (conf->adcOffset < -2047)
		conf->adcOffset = -2047;
   */
	// Offset subtrahieren von empfangssignal
	for (int16_t i = 0; i < EMPF_ARRAY_SIZE; i++) {
		conf->empfangssignal[i] -= conf->adcOffset;
	}
}

//=============================================//
// This function applies an incoming
// convolution operator (sendersignal) to an incoming set of
// data (empfangssignal) and deposits the filtered data in an
// output array (correlationssignal) whose reference is received as an
// incoming parameter.
// It calcualtes the positive or negative magnitude of the correlationsignal and determine the filter quality
// The correlationsignal is normally not used (I use it for second max/min) and if one need more variable space it can be comment.
#define SIDELOBE_CELLS 40
void cal_convolve(CAL_ChannelTypeDef *conf) {

	//int32_t zeit = millis();

	int32_t sumMax = 0;  // max correlation sum
	int16_t posMax = 0;  // position of max correlation sum
	int32_t sumMin = 0;
	int16_t posMin = 0;
	int32_t sumMax1 = 0;
	int16_t posMax1 = 0;
	int32_t sumMin1 = 0;
	int16_t posMin1 = 0;

	// Correlationssignal berechnen sowie minimum und maximum suchen
	// ------------------------------------------------------

	int32_t sum = 0;

	for (int16_t i = 0; i < CORELLATION_ARRAY_SIZE; i++) {
		sum = 0;
		for (int16_t j = 0; j < SENDER_ARRAY_SIZE; j++) {
			sum += sendersignal[j] * conf->empfangssignal[i + j];
		}  //end inner loop

		conf->correlationsignal[i] = sum;

		if (sum > sumMax) {
			sumMax = sum;
			posMax = i;
		}
		if (sum < sumMin) {
			sumMin = sum;
			posMin = i;
		}

	}  //end outer loop

	// Links und rechts vom Maximum das zweite Maximum suchen (bzw.minimum). sollte im bereich 480 liegen
	// bei +-GUARDCELLS von Maximumposition beginnen, da werte neben maximum größer sein können als zweites maximum
	// nicht weiter entfernt beginnen, da Störsignal maxima gleich nebenan haben kann
	int16_t i;
	if (sumMax > -sumMin) {
		if (posMax - SIDELOBE_CELLS > 0) {
			for (i = 0; i < posMax - SIDELOBE_CELLS; i++) {
				if (conf->correlationsignal[i] > sumMax1) {
					sumMax1 = conf->correlationsignal[i];
					posMax1 = i;
				}
			}
		}
		if (posMax + SIDELOBE_CELLS < CORELLATION_ARRAY_SIZE) {
			for (i = posMax + SIDELOBE_CELLS; i < CORELLATION_ARRAY_SIZE; i++) {
				if (conf->correlationsignal[i] > sumMax1) {
					sumMax1 = conf->correlationsignal[i];
					posMax1 = i;
				}
			}
		}
		conf->filterQuality = ((float) sumMax) / ((float) -sumMin);
		if(sumMax>sumMax1)
		  conf->magnitude = sumMax;
		else
		  conf->magnitude = sumMax1;

		//conf->magnitude = (float)conf->magnitude * 32000.0f / (float)maxCorelSumValue; // normalize to 32000
		conf->peakDistanz = abs(posMax - posMax1);

	} else {
		if (posMin - SIDELOBE_CELLS > 0) {
			for (i = 0; i < posMin - SIDELOBE_CELLS; i++) {
				if (conf->correlationsignal[i] < sumMin1) {
					sumMin1 = conf->correlationsignal[i];
					posMin1 = i;
				}
			}
		}
		if (posMin + SIDELOBE_CELLS < CORELLATION_ARRAY_SIZE) {
			for (i = posMin + SIDELOBE_CELLS; i < CORELLATION_ARRAY_SIZE; i++) {
				if (conf->correlationsignal[i] < sumMin1) {
					sumMin1 = conf->correlationsignal[i];
					posMin1 = i;
				}
			}
		}
		conf->filterQuality = ((float) -sumMin) / ((float) sumMax);

		if(sumMin<sumMin1)
		  conf->magnitude = sumMin;
		else
		  conf->magnitude = sumMin1;

		//conf->magnitude = (float)conf->magnitude * 32000.0f / (float)maxCorelSumValue; // normalize to 32000
		conf->peakDistanz = abs(posMin - posMin1);
	}

	// Nur wenn peak distantz im intervall liegt
	if (conf->peakDistanz > SENDER_ARRAY_SIZE - 3 && conf->peakDistanz < SENDER_ARRAY_SIZE + 3) {

		// alles ok
		if (conf->filterQuality > QUALITYTHRESHOLD) { // ist die qualität ok
			//digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
		} else {
			//digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
			conf->magnitude = 0;
		}
	} else {
		conf->magnitude = 0;
		conf->filterQuality = 0;
		//digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
	}

}      //end convolve method

//Empfangssignal und Korrelatiossignal auf Konsole ausgeben
void cal_printSignal(CAL_ChannelTypeDef *conf, int peakhigh, bool *flagShowSignal) {
	int i;
	char msg[100];

	// Peak ausgeben, um auf Graphen die Sampleabschnitte zu sehen.
	sprintf(msg, "%i\r\n", peakhigh);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	// Wieder zurücksetzen so dass graph nicht oben bleibt.
	sprintf(msg, "%i\r\n", 0);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	for (i = 0; i < CORELLATION_ARRAY_SIZE; i++) {
		if (*flagShowSignal) {
			int q = (int) (conf->filterQuality * 10.0f);
			sprintf(msg, "%i,%li,%i,%i\r\n", conf->empfangssignal[i], conf->correlationsignal[i], conf->peakDistanz, q);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);
			HAL_Delay(10);
		}
	}
}

//Empfangssignal auf Konsole ausgeben
void cal_printADCSignal(CAL_ChannelTypeDef *conf, int peakhigh, bool *flagShowSignal) {
	int i;
	char msg[100];

	// Peak ausgeben, um auf Graphen die Sampleabschnitte zu sehen.
	sprintf(msg, "%i\r\n", peakhigh);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	// Wieder zurücksetzen so dass graph nicht oben bleibt.
	sprintf(msg, "%i\r\n", 0);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	for (i = 0; i < EMPF_ARRAY_SIZE; i++) {
		if (*flagShowSignal) {
			sprintf(msg, "%i\r\n", conf->empfangssignal[i]);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			HAL_Delay(10);
		}
	}

}

//Offset auf Konsole ausgeben
void cal_printOffset(CAL_ChannelTypeDef *confL, CAL_ChannelTypeDef *confR, CAL_ChannelTypeDef *confB) {
	char msg[50];

	sprintf(msg, "L: %li  R: %li  B: %li\r\n", confL->adcOffset, confR->adcOffset, confB->adcOffset);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

}

