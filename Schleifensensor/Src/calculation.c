/*
 * calculation.c
 *
 *  Created on: 17.07.2016
 *      Author: Kai
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



	// Correlationssignal berechnen sowie minimum und maximum suchen
	// ------------------------------------------------------

	int32_t sum = 0;

	for (int16_t i = 0; i < CORELLATION_ARRAY_SIZE; i++) {
		sum = 0;
		for (int16_t j = 0; j < SENDER_ARRAY_SIZE; j++) {
			sum += sendersignal[j] * conf->empfangssignal[i + j];
		}  //end inner loop
		conf->correlationsignal[i] = sum;///10; //=> 90000/50 = 1800  1800*1800 = 3.240.000
	}  //end outer loop


	// ---- square correlation array ---------
    double result;
	for (int i = 0; i < CORELLATION_ARRAY_SIZE; i++) {
		if (conf->correlationsignal[i] < 0){
			result = -1*pow((double)conf->correlationsignal[i],2);
			conf->correlationsignal[i] = result/100000;
	    }
		else{
			result = pow((double)conf->correlationsignal[i],2);
			conf->correlationsignal[i] =  result/100000;
		}
	}


	// ---- find peak ---------
	conf->peakValue = 0;
	conf->peakIdx = 0;
	for (int i = 0; i < CORELLATION_ARRAY_SIZE; i++) {
		if (abs(conf->correlationsignal[i]) > abs(conf->peakValue)) {
			conf->peakValue = conf->correlationsignal[i]; //maximal Signalintensität
			conf->peakIdx = i;
		}
	}


	// --- get correlation sum (without max peak and nearby coils) and peakValue2 value -------
	//quadratische Mittelwert QMW - mean squared error
	conf->corrSum = 0;
	conf->peakValue2 = 0;
	conf->peakIdx2 = 0;
	int count = 0;
	if (conf->peakIdx > CORELLATION_ARRAY_SIZE - SIDELOBE_CELLS) {
		for (int i = SIDELOBE_CELLS; i < (conf->peakIdx - SIDELOBE_CELLS); i++) {
			conf->corrSum += pow(conf->correlationsignal[i],2);
			count++;
			if (abs(conf->correlationsignal[i]) > abs(conf->peakValue2)) {
				conf->peakValue2 = conf->correlationsignal[i]; //maximal Signalintensität
				conf->peakIdx2 = i;
			}
		}
	}
	else  if (conf->peakIdx < SIDELOBE_CELLS) {
		for (int i = conf->peakIdx + SIDELOBE_CELLS; i < (CORELLATION_ARRAY_SIZE - SIDELOBE_CELLS); i++) {
			conf->corrSum += pow(conf->correlationsignal[i],2);
			count++;
			if (abs(conf->correlationsignal[i]) > abs(conf->peakValue2)) {
				conf->peakValue2 = conf->correlationsignal[i]; //maximal Signalintensität
				conf->peakIdx2 = i;
			}
		}
	}
	else {
		for (int i = 0; i < CORELLATION_ARRAY_SIZE; i++) {
			if (i < (conf->peakIdx - SIDELOBE_CELLS) || i >(conf->peakIdx + SIDELOBE_CELLS)) {
				conf->corrSum += pow(conf->correlationsignal[i],2);
				count++;
				if (abs(conf->correlationsignal[i]) > abs(conf->peakValue2)) {
					conf->peakValue2 = conf->correlationsignal[i]; //maximal Signalintensität
					conf->peakIdx2 = i;
				}
			}
		}
	}


	if(count == 0) count = 1;

	conf->MSE = (float)conf->corrSum / ((float)count);

	if (conf->MSE > 0.000001f) {
		conf->psnr = (pow(((float)(conf->peakValue)),2) / conf->MSE);
		conf->psnr2 = (pow(((float)(conf->peakValue2)),2) / conf->MSE);
	}
	else {
		conf->psnr = 1;
		conf->psnr2 = 1;
	}

	if (conf->psnr2 < 0.000001f) {
		conf->psnr2 = 0.000001f;
	}
	conf->ratio = conf->psnr / conf->psnr2;


   if ( conf->ratio > 2.5f && conf->psnr > 25 && (abs(conf->peakValue) > 1000)) {
	   conf->magnitude = conf->peakValue;
	   conf->filterQuality = conf->psnr;
	}

}      //end convolve method

//Empfangssignal und Korrelatiossignal auf Konsole ausgeben
void cal_printSignal(CAL_ChannelTypeDef *conf, int peakhigh, bool *flagShowSignal) {
	int i;
	char msg[100];
/*
	// Peak ausgeben, um auf Graphen die Sampleabschnitte zu sehen.
	sprintf(msg, "%i\r\n", peakhigh);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	// Wieder zurücksetzen so dass graph nicht oben bleibt.
	sprintf(msg, "%i\r\n", 0);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
*/
	for (i = 0; i < CORELLATION_ARRAY_SIZE; i++) {
		if (*flagShowSignal) {
			sprintf(msg, "%i,%li,%d,%f\r\n", conf->empfangssignal[i], conf->correlationsignal[i],  (int)conf->peakValue, conf->filterQuality);
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

