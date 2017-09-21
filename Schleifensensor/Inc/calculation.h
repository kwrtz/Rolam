/*
 * calculation.h
 *
 *  Created on: 17.07.2016
 *      Author: Kai
 */

#include "stm32f4xx_hal.h"

#ifndef APPLICATION_USER_CALCULATION_H_
#define APPLICATION_USER_CALCULATION_H_

#define SENDER_ARRAY_SIZE  (480) //480 328 //240 // 92 96  //Bosch=328 pseudonoise4_dbpsk=240  Ardumower = 96
#define EMPF_ARRAY_SIZE   (SENDER_ARRAY_SIZE*2) //720 //288  //=> Signal wird mindestens 2x im Abtastintervall erkannt
#define CORELLATION_ARRAY_SIZE  (EMPF_ARRAY_SIZE - SENDER_ARRAY_SIZE + 1)
#define QUALITYTHRESHOLD   (1.49f) //Bosch=1.2 pseudonoise4_dbpsk = 1.5

typedef enum {
	false = 0, true = 1
} bool;

typedef struct {

	int16_t * empfangssignal;

	int32_t adcOffset;

	int32_t peakValue;
	int32_t peakIdx;
	int32_t peakValue2;
	int32_t peakIdx2;
	int64_t corrSum;
	float MSE;
	float psnr;
	float psnr2;
	float ratio;


	//Ergebnis per Aufruf von convolve()
	int32_t magnitude;         // Amplitude des Korrelationssignals
	float filterQuality; // Verh�ltnis maxAmplitude zu minAmplitude immer Positive


	// Wenn Spule andersherum angeschlossen ist
	//bool swapCoilPolarity = false;

	int32_t correlationsignal[CORELLATION_ARRAY_SIZE]; // Wird von convolve ausgef�llt. Wird benutzt um zweites maximum zu finden

} CAL_ChannelTypeDef;

extern void cal_init();
extern void cal_offset(CAL_ChannelTypeDef *conf);
extern void cal_convolve(CAL_ChannelTypeDef *conf);

extern void cal_printSignal(CAL_ChannelTypeDef *conf, int peakhigh, bool *flagShowSignal);
extern void cal_printADCSignal(CAL_ChannelTypeDef *conf, int peakhigh, bool *flagShowSignal);
extern void cal_printOffset(CAL_ChannelTypeDef *confL, CAL_ChannelTypeDef *confR, CAL_ChannelTypeDef *confB);

#endif /* APPLICATION_USER_CALCULATION_H_ */
