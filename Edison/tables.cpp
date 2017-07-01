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

#include "tables.h"

bool  table_b[TABLE_B_END];  // 0 = false, 1 = true
int   table_i[TABLE_I_END];
float table_f[TABLE_F_END];


void tables_init() {

	SETTB(TB_ACTVATE_AUTO_SPIRAL, 1);
	SETTB(TB_SHOW_ROTATE, 0);




	SETTF(TF_ENCTICKSPERROTATION, 2120.0f);  // Anzahl encoder flanken pro umdrehung - positive und negative flanken.
	SETTF(TF_RADUMFANG_CM, 80.738f); // Radumfang (circumference) in cm 78.54f 81,5
	SETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED, 1140.0f); // maximale Encoder Geschwindigkeit bei voller Motordrezahl (ticks pro sek).
	SETTF(TF_MAX_WHEEL_RPM, 32.62f); // max rounds per  minute the wheel should reach when speed is 100% 
	SETTF(TF_DISTANCE_BETWEEN_WHEELS_CM, 37.0f);
	SETTF(TF_MAX_SPIRAL_RADIUS_CM, 150.0f);
	SETTF(TF_START_SPIRAL_RADIUS_CM, 27.0f);
	SETTF(TF_SPIRAL_SEGMENTS, 16.0f);


}
