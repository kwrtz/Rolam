/*
 * send.c
 *
 *  Created on: 18.07.2016
 *      Author: Kai
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "calculation.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//--------------------------------------------------
// Send result over serial line to master controller
//--------------------------------------------------

union uFloat {
	uint8_t uBytes[4];
	float sFloat;
};

union uInt32 {
	uint8_t uBytes[4];
	int32_t sIn32t;
};

union uInt16 {
	uint8_t uBytes[2];
	int16_t sIn16t;
};

union uInt8 {
	uint8_t uBytes[1];
	int8_t sInt8;
};

union uBool {
	uint8_t uBytes[1];
	bool uBool;
};

/**************************************************************************/
/*!
 Print one char to serial line
 */
/**************************************************************************/
void print3(uint8_t c) {
	HAL_UART_Transmit(&huart3, (uint8_t*) &c, 1, HAL_MAX_DELAY);
}

// If I would scale the magnitude to 32000 then the divider must be 14.8
// See calculation.c //conf->magnitude = (float)conf->magnitude * 32000.0f / (float)maxCorelSumValue; // normalize to 32000
// But in order the full magnitude will never be reached, I divide the magnitude by 10.
int16_t scaleMagnetude(long _magnitude) {

	// magnitude könnte größer als int16_t max werden.
	long lMagnitude = _magnitude / 10;

	// check overflow
	if (lMagnitude > 32000)
		lMagnitude = 32000;
	else if (lMagnitude < -32000)
		lMagnitude = -32000;

	// don't let magnetude be 0 if _magnitude is not. Because 0 means no valid signal
	if (lMagnitude == 0 && _magnitude != 0) {
		if (_magnitude > 0)
			lMagnitude = 1;
		if (_magnitude < 0)
			lMagnitude = -1;
	}

	return (int16_t) lMagnitude;
}

int16_t scaleMagnetudeBackCoil(long _magnitude) {

	// magnitude könnte größer als int16_t max werden.
	long lMagnitude = _magnitude / 5;

	// check overflow
	if (lMagnitude > 32000)
		lMagnitude = 32000;
	else if (lMagnitude < -32000)
		lMagnitude = -32000;

	// don't let magnetude be 0 if _magnitude is not. Because 0 means no valid signal
	if (lMagnitude == 0 && _magnitude != 0) {
		if (_magnitude > 0)
			lMagnitude = 1;
		if (_magnitude < 0)
			lMagnitude = -1;
	}

	return (int16_t) lMagnitude;
}


void sendToMaster(CAL_ChannelTypeDef *confL, CAL_ChannelTypeDef *confR, CAL_ChannelTypeDef *confB, bool flag_ShowValuesSendToMaster) {

	// Header Bytes. Sollten in den Uebertragenen Daten kein weiteres mal so vorkommen.
	uint8_t header1 = 0xFF; //Header
	uint8_t header2 = 0xFF; //Header


	// Sensor links
	union uInt16 magnetudeL;
	magnetudeL.sIn16t = scaleMagnetude(confL->magnitude);
	//magnetudeL.sIn16t=confL->magnitude;

	// Sensor Rechts
	union uInt16 magnetudeR;
	magnetudeR.sIn16t = scaleMagnetude(confR->magnitude);
	//magnetudeR.sIn16t = confR->magnitude;

	// Sensor Rechts
	union uInt16 magnetudeB;
	magnetudeB.sIn16t = scaleMagnetudeBackCoil(confB->magnitude);
	//magnetudeB.sIn16t = confB->magnitude;


	//checksumm
	uint8_t checksumm = header1 + header2;
	checksumm += magnetudeL.uBytes[0] + magnetudeL.uBytes[1];
	checksumm += magnetudeR.uBytes[0] + magnetudeR.uBytes[1];
	checksumm += magnetudeB.uBytes[0] + magnetudeB.uBytes[1];
	checksumm &= 0x7F; //B01111111;

	//send
	print3(header1);
	print3(header2);
	//
	print3(magnetudeL.uBytes[0]);
	print3(magnetudeL.uBytes[1]);
	//
	print3(magnetudeR.uBytes[0]);
	print3(magnetudeR.uBytes[1]);
	//
	print3(magnetudeB.uBytes[0]);
	print3(magnetudeB.uBytes[1]);
	//
	print3(checksumm);

	/*
	 debug.print("ML0: ") ;
	 debug.println(magnetudeL.uBytes[0]) ;
	 debug.print("ML1: ") ;
	 debug.println(magnetudeL.uBytes[1]) ;
	 */
	if (flag_ShowValuesSendToMaster) {
		char msg[200];
		sprintf(msg, "ML: %i MR: %i MB: %i QL: %f QR: %f QB: %f RATL: %f RATR: %f RATB: %f\r\n", magnetudeL.sIn16t, magnetudeR.sIn16t, magnetudeB.sIn16t,
				confL->filterQuality, confR->filterQuality, confB->filterQuality,  confL->ratio, confR->ratio , confB->ratio);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	}



}
