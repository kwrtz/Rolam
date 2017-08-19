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


#ifndef _POSITIONCONTROL_h
#define _POSITIONCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



#include "Thread.h"
#include "PID_v2.h"
#include "helpers.h"
#include "Sabertooth.h"
#include "closedloopcontrol.h"
#include "CRotaryEncoder.h"



enum EPosState
{
	STP_OFF = -1, // optional, -1 is the initial state of the fsm
	STP_START_CM,
	STP_DRIVETOPOSITION,
	STP_TARGETREACHED,
	STP_STOP//,STP_STOPPOSITIONING
};

class TPositionControl : public Thread, public FSM<EPosState>
{


public:


	virtual void run();

	void setup(TClosedLoopControlThread *_motor, CRotaryEncoder *enc);

	void rotateAngle(float _angle, long _speedPercentage);
	void rotateCM(float _cm, long _speedPercentage);
	//void stopPositioning();
	void reset();
	bool isPositionReached(); // rotateForward oder rotateForward fertig mit ausführugn?

							  // Wegberechnungsroutinen
	long getCountsForCM(float x);
	long getCountsForDegree(float x);
	float getDegreeForCounts(float x);
	float getCMForCounts(float x);

	bool flagShowResults;

	float posKp;
	float stopCmBeforeTarget;
	float addCmToTargetPosition;

	void showConfig();

private:



	float startPositionCM;
	float sollPositionCM;
	float inputPositionCM;
	long  sollSpeedPercentage; // Soll speed wird von setSpeed gesetzt 

	virtual void UpdateState(EPosState t);

	TClosedLoopControlThread *motor;
	CRotaryEncoder *myEncoder;



};

#endif

