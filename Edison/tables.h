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

#ifndef _TABLES_h
#define _TABLES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



#define TB_ACTVATE_AUTO_SPIRAL  0
#define TB_SHOW_ROTATE  1  // Show values from rotation function

#define TABLE_B_END  3
//-----------------------------------------------------------------------------------------------------

#define TABLE_I_END  3


//-----------------------------------------------------------------------------------------------------
#define TF_ENCTICKSPERROTATION                  0  // Anzahl encoder flanken pro umdrehung - positive und negative flanken.
#define TF_RADUMFANG_CM                         1 
#define TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED   2  // maximale Geschwindigkeit (ticks pro sek). Motor erreicht eine Geschwindigkeit von 1470 Ticks/sek bei motorpower von 127. Wird in Funktion setspeed verwendet. 1140 bedeuten 100%.
#define TF_MAX_WHEEL_RPM                        3  // max rounds per  minute the wheel should reach when speed is 100% 
#define TF_DISTANCE_BETWEEN_WHEELS_CM           4  // wheel-to-wheel distance (cm)
#define TF_MAX_SPIRAL_RADIUS_CM                 5  // After which Radius should spiral stop
#define TF_START_SPIRAL_RADIUS_CM               6  // With which Radius should spiral start
#define TF_SPIRAL_SEGMENTS                      7  //  How often should the calculation be done within 360 degrees. 16 Means every 22,5 degrees. This means that 360 is splited in 16 segments

#define TABLE_F_END  10


//*** ACHTUNG DIE GET FUBKTIONEN DÜRFEN NICHT IM CONSTRUCTOR VERWENDET WERDEN!!! *******

extern bool  table_b[TABLE_B_END];
extern int   table_i[TABLE_I_END];
extern float table_f[TABLE_F_END];

extern void tables_init();

inline bool  GETTB(int x) { return (table_b[x]); }
inline int   GETTI(int x) { return (table_i[x]); }
inline float GETTF(int x) { return (table_f[x]); }

inline void  SETTB(int x, bool value) { table_b[x] = value; }
inline void  SETTI(int x, int value) { table_i[x] = value; }
inline void  SETTF(int x, float value) { table_f[x] = value; }



#endif

