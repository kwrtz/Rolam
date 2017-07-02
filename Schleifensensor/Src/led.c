/*
 * led.c
 *
 *  Created on: 18.07.2016
 *      Author: Kai
 */

#include "stm32f4xx_hal.h"
#include "calculation.h"


static uint32_t lastTimeBlinkLOutside = 0;
static uint32_t lastTimeBlinkLNoSignal = 0;
static uint32_t lastTimeBlinkROutside = 0;
static uint32_t lastTimeBlinkRNoSignal = 0;

void ledSetState(CAL_ChannelTypeDef *confL, CAL_ChannelTypeDef *confR) {
	uint32_t now;


	now = HAL_GetTick();

	// left perimeter status
	if (confL->magnitude > 0) {
		if ((now-lastTimeBlinkLOutside) < 100ul) {
			;
		} else {
			lastTimeBlinkLOutside = HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOA, LDLEFT_Pin);
		}
	} else if (confL->magnitude < 0) {
		HAL_GPIO_WritePin(GPIOA, LDLEFT_Pin, GPIO_PIN_SET);
	} else {
		if ((now-lastTimeBlinkLNoSignal) < 1000ul) {
		} else {
			lastTimeBlinkLNoSignal = HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOA, LDLEFT_Pin);
		}
	}

	// right perimeter status
	if (confR->magnitude > 0) {
		if ((now-lastTimeBlinkROutside) < 100ul) {
			;
		} else {
			lastTimeBlinkROutside = HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOA, LDRIGHT_Pin);
		}
	} else if (confR->magnitude < 0) {
		HAL_GPIO_WritePin(GPIOA, LDRIGHT_Pin, GPIO_PIN_SET);
	} else {
		if ((now-lastTimeBlinkRNoSignal) < 1000ul) {
		} else {
			lastTimeBlinkRNoSignal = HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOA, LDRIGHT_Pin);
		}
	}

}
