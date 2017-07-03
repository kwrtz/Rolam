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

#include "mowclosedloopcontrol.h"
#include "hardware.h"
#include "errorhandler.h"





void TMowClosedLoopControlThread::setup(uint8_t motorNumber)    // Motor 1 oder 2
{
	// Welcher motor wird am Sabertooth angesteuert 1 oder 2
	motorNo = motorNumber;
	SetState(STMM_STOP);
	motorDisabled = false;
	uiMotorDisabled = false;
	speedCurr = 0;
	motorMowAccel = 2000;
}


/*********************************************************/
// Motor FSM ausführen
/*********************************************************/
void TMowClosedLoopControlThread::run()
{
	// Wird alle 200ms aufgerufen

	runned();
	LoopFSM();

}

/*********************************************************/
// Motor Zustände Do State
/*********************************************************/
void TMowClosedLoopControlThread::UpdateState(EMowMotorState t)
{

	// Ramp up until 127
	speedCurr += (interval * (127 - speedCurr)) / motorMowAccel; // intertval comes from thread
	if (speedCurr < 30)
		speedCurr = 30;

	switch (t) {
	case STMM_FORWARD:
		if (motorDisabled || uiMotorDisabled) {
			mowMotorDriver.motor(motorNo, 0);
			speedCurr = 0;
		}
		else {

			mowMotorDriver.motor(motorNo, speedCurr);
		}
		break;

	case STMM_BACKWARD:
		if (motorDisabled || uiMotorDisabled) {
			mowMotorDriver.motor(motorNo, 0);
			speedCurr = 0;
		}
		else {
			mowMotorDriver.motor(motorNo, -speedCurr);
		}
		break;

	case STMM_STOP:
		mowMotorDriver.motor(motorNo, 0);
		speedCurr = 0;
		break;

	default:
		//TODO invalid state - reset, perhaps?
		break;
	}

};



void TMowClosedLoopControlThread::forward()
{

	if (GetState() == STMM_BACKWARD) {
		errorHandler.setInfo(F("FORWARD not possible. STOP MOTOR FIRST!!!\r"));
		return;
	}
	SetState(STMM_FORWARD);
}


void TMowClosedLoopControlThread::backward()
{

	if (GetState() == STMM_FORWARD) {
		errorHandler.setInfo(F("BACKWARD not possible. STOP MOTOR FIRST!!!\r"));
		return;
	}
	SetState(STMM_BACKWARD);
}


bool  TMowClosedLoopControlThread::isRunning()
{

	return (GetState() == STMM_FORWARD || GetState() == STMM_BACKWARD);
}


void TMowClosedLoopControlThread::stop()
{
	//SetState(STMM_STOP_REQUEST);
	SetState(STMM_STOP);
}


bool TMowClosedLoopControlThread::isStopped()
{
	return (GetState() == STMM_STOP);
}

// ---------------------------------------------------------
// Sabnertooth settings only needed one time for configuration of sabertooth
// ---------------------------------------------------------

void TMowClosedLoopControlThread::setRamp()
{
	//The Sabertooth remembers this command between restarts AND in all modes.
	//  mowMotorDriver.setRamping(1); // 14 ca 4 sek   18 ca. 2sek
}

void TMowClosedLoopControlThread::setBaudRate()
{
	//The Sabertooth remembers this command between restarts AND in all modes.
	//  mowMotorDriver.setBaudRate(19200);
}

void TMowClosedLoopControlThread::controlDirect(int speed)
{

	if (speed < -127) speed = -127;
	else if (speed > 127) speed = 127;

	mowMotorDriver.motor(motorNo, speed);
}


