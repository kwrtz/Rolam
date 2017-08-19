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

#ifndef _MOTOR_h
#define _MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "helpers.h"
#include "mowclosedloopcontrol.h"
#include "closedloopcontrol.h"
#include "positioncontrol.h"



class TMotorInterface
{
private:


	// Distance measurement for different functions
	long encCountsLWorkx;
	long encCountsRWorkx;
	unsigned long lastRunDistanceMeasurementWorkx;

	long encCountsLAreaX;
	long encCountsRAreaX;
	unsigned long lastRunDistanceMeasurementAreaX;

	bool coilOutMeasurementRunningL;
	bool coilOutMeasurementRunningR;
	long encCountsLCoilOut;
	long encCountsRCoilOut;
	unsigned long lastRunDistanceMeasurementCoilOut;
	unsigned long lastRunDistanceMeasurementCoilAngle;

	long encCountsLSpiral;
	unsigned long lastRunDistanceMeasurementSpiral;

public:

	TClosedLoopControlThread *L;
	TClosedLoopControlThread *R;
	TMowClosedLoopControlThread *M;
	TPositionControl *pcL;
	TPositionControl *pcR;

	void stopAllMotors();

	// Mow motor
	void mowMotStart();
	void mowMotStop();
	bool isMowMotRunning();

	// drive motors
	void setSpeed(long  speed); //-100% to +100%
	void stop(); // stop drive motors
	void hardStop();
	bool isStopped();

	void enableDefaultRamping();
	void enablePerTrackRamping();
	void enableFastStopRamping();


	// positioning routines
	void rotateAngle(float angle, long _speed);
	void rotateCM(float _cm, long _speed);
	void turnTo(float angle, long _speed);
	//void stopPositioning(); // query isPositionReached() to determine if motors are stopped!!!
	bool isPositionReached();

	// Calculate Encodercounts to Meter
	long getCountsForM(float x);
	long getMForCounts(float x);


	void resetEncoderCounter();
	void startDistanceMeasurementCoilOut(bool b); // b=true wenn beide cols draußen sind
	void stopDistanceMeasurementLCoilOut();
	void stopDistanceMeasurementRCoilOut();
	float getDistanceDiffInCMForCoilOut();
	float getDistanceAngleCoilOut();

	void startDistanceMeasurementAreax();
	void startDistanceMeasurementWorkx();
	void startDistanceMeasurementSpiral();
	long getDistanceInMeterAreax();
	long getDistanceInCMForWorkx();
	float getDistanceLInCMSpiral();


	bool flagShowDistance; // When GotoAreaX is activated, showing the distance can be switched on with this flag.



	void setup(TMowClosedLoopControlThread *_M, TClosedLoopControlThread *_LEFT, TClosedLoopControlThread *_R, TPositionControl *_pcL, TPositionControl *_pcR);


	void testPosForwardStopBackward();
};



#endif

