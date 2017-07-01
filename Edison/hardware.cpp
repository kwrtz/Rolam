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

#include "hardware.h"



#define LED1 13
//#define SRF08_SDA_Pin PB_9
//#define SRF08_SCL_Pin PB_8


#define BATVOLTAGE_Pin A0 
#define MOWMOTCURRENT_Pin A1 

#define pinCHARGEVOLTAGE A2 
#define pinCHARGECURRENT A3 
#define pinCHARGINGENABLE 53 



//#define BUMPERL_Pin PC_10 
//#define BUMPERR_Pin PC_12 

#define MPX5010DP_Pin A4


#define ENCODERLEFT_A_Pin 53 
#define ENCODERLEFT_B_Pin 51 
#define ENCODERRIGTH_A_Pin 49 
#define ENCODERRIGTH_B_Pin 47

#define RANDOM_Pin A11 


BufferSerial pc(Serial, 256);
BufferSerial bt(Serial1, 256);
BufferSerial per(Serial2, 256);
BufferSerial &debug = pc;
//BufferSerial &debug = bt;
BufferSerial &perRX = per;
BufferSerial &sabertoothTX = per;


//SRF08 rangeMod1(SRF08_SDA_Pin, SRF08_SCL_Pin, 0xE2); //SRF08 ranging module 1 (PinName SDA, PinName SCL, int i2cAddress)


AnalogIn aiBATVOLT(BATVOLTAGE_Pin);
AnalogIn aiMOWMOTCURRENT(MOWMOTCURRENT_Pin);
AnalogIn aiCHARGEVOLTAGE(pinCHARGEVOLTAGE);
AnalogIn aiCHARGECURRENT(pinCHARGECURRENT);

DigitalOut doChargeEnable(pinCHARGINGENABLE);


AnalogIn aiRandomIn(RANDOM_Pin); // Pin für die srand Funktion



DigitalIn diEncLA(ENCODERLEFT_A_Pin,true);
DigitalIn diEncLB(ENCODERLEFT_B_Pin, true);
DigitalIn diEncRA(ENCODERRIGTH_A_Pin, true);
DigitalIn diEncRB(ENCODERRIGTH_B_Pin, true);

CRotaryEncoder encoderL(diEncLA, diEncLB);
CRotaryEncoder encoderR(diEncRA, diEncRB);

DigitalOut doMyLED(LED1);

//DigitalIn diBumperL(BUMPERL_Pin,PullUp);
//DigitalIn diBumperR(BUMPERR_Pin,PullUp);

AnalogIn aiBumper(MPX5010DP_Pin);

static void ISR_ML_ENC_SIGA() {
	encoderL.rise();
}

static void ISR_MR_ENC_SIGA() {
	encoderR.rise();
}


void hardwareSetup() {

	pc.serial.begin(115200);
	bt.serial.begin(921600);
	per.serial.begin(19200);

	encoderL.isReversed();

	attachInterrupt(digitalPinToInterrupt(ENCODERLEFT_A_Pin), ISR_ML_ENC_SIGA, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODERRIGTH_A_Pin), ISR_MR_ENC_SIGA, RISING);


}