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

#ifndef _HARDWARE_h
#define _HARDWARE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "DigitalInOut.h"
#include "BufferSerial.h"
#include "CRotaryEncoder.h"
#include "Sabertooth.h"
//#include "SRF08.h"


extern BufferSerial &debug;
extern BufferSerial &perRX;
//extern BufferSerial &sabertoothTX;


extern Sabertooth motordriver;
extern Sabertooth mowMotorDriver;

//extern SRF08 rangeMod1;

extern AnalogIn aiBATVOLT;
extern AnalogIn aiMOWMOTCURRENT;

extern AnalogIn aiCHARGEVOLTAGE;
extern AnalogIn aiCHARGECURRENT;

extern DigitalOut doChargeEnable;


extern AnalogIn aiRandomIn;

extern CRotaryEncoder encoderL;
extern CRotaryEncoder encoderR;


extern DigitalOut doMyLED;

//extern DigitalIn diBumperL;
//extern DigitalIn diBumperR;
extern AnalogIn aiBumper;

extern void hardwareSetup();

#endif

