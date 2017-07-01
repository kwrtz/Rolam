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
