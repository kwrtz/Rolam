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

#include "motor.h"
#include "hardware.h"
#include "errorhandler.h"
#include "tables.h"

//#define ENCODER_TEST 1   // use this to verify that your encoders are working


/*********************************************************************************************
Klassenfunktionen.
********************************************************************************************/

void TMotorInterface::setup(TMowClosedLoopControlThread *_M, TClosedLoopControlThread *_LEFT, TClosedLoopControlThread *_R, TPositionControl *_pcL, TPositionControl *_pcR)
{
	L = _LEFT;
	R = _R;
	pcL = _pcL;
	pcR = _pcR;
	M = _M;

	flagShowDistance = false;
}

void TMotorInterface::stopAllMotors()
{
	//stopPositioning();
	stop();
	mowMotStop();
}

void TMotorInterface::hardStop()
{
	L->hardStop();
	R->hardStop();
	pcL->reset();
	pcR->reset();
}

void TMotorInterface::stop()
{
	L->stop();
	R->stop();
	pcL->reset();
	pcR->reset();
}

void TMotorInterface::mowMotStart()
{
	M->forward();
}

void TMotorInterface::mowMotStop()
{
	M->stop();
}

bool TMotorInterface::isMowMotRunning()
{
	return M->isRunning();
}



void TMotorInterface::setSpeed(long  speed)
{
	L->setSpeed(speed);
	R->setSpeed(speed);
}

void TMotorInterface::enableDefaultRamping()
{
	L->enableDefaultRamping();
	R->enableDefaultRamping();
}
void TMotorInterface::enablePerTrackRamping()
{
	L->enablePerTrackRamping();
	R->enablePerTrackRamping();
}

void TMotorInterface::enableFastStopRamping()
{
	L->enableFastStopRamping();
	R->enableFastStopRamping();
}

/*
void TMotorInterface::stopPositioning(){
pcL->stopPositioning();
pcR->stopPositioning();
}
*/

void TMotorInterface::rotateAngle(float _angle, long _speed)
{
	pcL->rotateAngle(_angle, _speed);
	pcR->rotateAngle(_angle, _speed);
}

void TMotorInterface::rotateCM(float _cm, long _speed)
{
	pcL->rotateCM(_cm, _speed);
	pcR->rotateCM(_cm, _speed);
}

void TMotorInterface::turnTo(float _angle, long _speed)
{
	//float weg = (_angle * PI * (GETTF(TF_DISTANCE_BETWEEN_WHEELS_CM)/2) ) /180;
	float weg = (_angle * PI * GETTF(TF_DISTANCE_BETWEEN_WHEELS_CM)) / 360;

	pcL->rotateCM(weg, _speed);
	pcR->rotateCM(-1 * weg, _speed);

}

bool  TMotorInterface::isPositionReached()
{

	if (pcL->isPositionReached() && pcR->isPositionReached()) {
		return true;
	}
	return false;
}

bool  TMotorInterface::isStopped()
{
	if (L->GetState() == STM_STOP && R->GetState() == STM_STOP) {
		return true;
	}
	return false;
}

long TMotorInterface::getCountsForM(float x)
{
	return pcL->getCountsForCM(x * 100);
}


long TMotorInterface::getMForCounts(float x)
{
	return pcL->getCMForCounts(x) / 100;
}


void TMotorInterface::resetEncoderCounter()
{
	L->myEncoder->resetTickCounter();
	R->myEncoder->resetTickCounter();
	L->myEncoder->resetAbsTicksCounter();
	R->myEncoder->resetAbsTicksCounter();
}

long encCountsLCoilOut;
long encCountsRCoilOut;
unsigned long lastRunDistanceMeasurementCoilOut;

void TMotorInterface::startDistanceMeasurementCoilOut(bool b)
{
	if (b) { // Beide coils outside
		encCountsLCoilOut = L->myEncoder->getTickCounter();
		encCountsRCoilOut = R->myEncoder->getTickCounter();
		lastRunDistanceMeasurementCoilOut = millis();
		coilOutMeasurementRunningL = true;
		coilOutMeasurementRunningR = true;
	}
	else { // Wenn nur eine coil outside, hohe distanz vorgeben
		encCountsLCoilOut = pcL->getCountsForCM(100);
		encCountsRCoilOut = 0;
		lastRunDistanceMeasurementCoilOut = millis();
		coilOutMeasurementRunningL = false;
		coilOutMeasurementRunningR = false;
	}

}

void TMotorInterface::stopDistanceMeasurementLCoilOut()
{
	if (coilOutMeasurementRunningL) {
		encCountsLCoilOut = L->myEncoder->getTickCounter() - encCountsLCoilOut;
		coilOutMeasurementRunningL = false;
	}
}

void TMotorInterface::stopDistanceMeasurementRCoilOut()
{
	if (coilOutMeasurementRunningR) {
		encCountsRCoilOut = R->myEncoder->getTickCounter() - encCountsRCoilOut;
		coilOutMeasurementRunningR = false;
	}
}


float TMotorInterface::getDistanceDiffInCMForCoilOut()
{
	if (coilOutMeasurementRunningL || coilOutMeasurementRunningR) { //Eine Spule nicht draußen gewesen
		return 100.0f;
	}

	long encCounts = encCountsLCoilOut - encCountsRCoilOut;
	encCounts = abs(encCounts);

	float difference = pcL->getCMForCounts(encCounts);
	difference = abs(difference);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementCoilOut  > 100) {
			lastRunDistanceMeasurementCoilOut = millis();
			sprintf(errorHandler.msg, "!03,DistCoilOutDiff CM: %f encCountsDiff %ld\r\n", difference, encCounts);
			errorHandler.setInfo();
		}
	}

	return difference;
}


float TMotorInterface::getDistanceAngleCoilOut()
{
	if (coilOutMeasurementRunningL || coilOutMeasurementRunningR) {
		return 90.0f;
	}

	long encCounts = encCountsLCoilOut - encCountsRCoilOut;
	encCounts = abs(encCounts);

	float difference = pcL->getCMForCounts(encCounts);
	difference = abs(difference);

	float angle = atan(difference / 26.5f) * 180 / PI; // 26.5cm Abstand der Spulen

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementCoilAngle  > 100) {
			lastRunDistanceMeasurementCoilAngle = millis();
			sprintf(errorHandler.msg, "!03,DistCoilOutAngle: %f encCountsDiff %ld\r\n", angle, encCounts);
			errorHandler.setInfo();
		}
	}

	return angle;
}


void TMotorInterface::startDistanceMeasurementWorkx()
{
	encCountsLWorkx = L->myEncoder->getTickCounter();
	encCountsRWorkx = R->myEncoder->getTickCounter();
	lastRunDistanceMeasurementWorkx = millis();
}

long TMotorInterface::getDistanceInCMForWorkx()
{
	long encCountsL = L->myEncoder->getTickCounter() - encCountsLWorkx;
	long encCountsR = R->myEncoder->getTickCounter() - encCountsRWorkx;

	if (encCountsL <0) encCountsL = 0;
	if (encCountsR <0) encCountsR = 0;

	long encCounts = encCountsL + encCountsR;
	encCounts /= 2;

	long drivenDistance = pcL->getCMForCounts(encCounts);
	drivenDistance = abs(drivenDistance);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementWorkx  > 100) {
			lastRunDistanceMeasurementWorkx = millis();
			sprintf(errorHandler.msg, "!03,DistWorkx CM: %ld encCounts %ld\r\n", drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}

	return drivenDistance;
}


void TMotorInterface::startDistanceMeasurementSpiral()
{
	encCountsLSpiral = L->myEncoder->getTickCounter();
	lastRunDistanceMeasurementSpiral = millis();
}


float TMotorInterface::getDistanceLInCMSpiral()
{
	long encCounts = L->myEncoder->getTickCounter() - encCountsLSpiral;;

	encCounts = abs(encCounts);

	float drivenDistance = pcL->getCMForCounts(encCounts);
	drivenDistance = abs(drivenDistance);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementSpiral  > 100) {
			lastRunDistanceMeasurementSpiral = millis();
			sprintf(errorHandler.msg, "!03,DistSpiralL CM: %f encCounts %ld\r\n", drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}

	return drivenDistance;
}



void TMotorInterface::startDistanceMeasurementAreax()
{
	encCountsLAreaX = L->myEncoder->getTickCounter();
	encCountsRAreaX = R->myEncoder->getTickCounter();
	lastRunDistanceMeasurementAreaX = millis();
}

long TMotorInterface::getDistanceInMeterAreax()
{
	long encCountsL = L->myEncoder->getTickCounter() - encCountsLAreaX;
	long encCountsR = R->myEncoder->getTickCounter() - encCountsRAreaX;

	encCountsL = abs(encCountsL);
	encCountsR = abs(encCountsR);


	long encCounts = encCountsL + encCountsR;
	encCounts /= 2;

	long drivenDistance = getMForCounts(encCounts);
	drivenDistance = abs(drivenDistance);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementAreaX  > 500) {
			lastRunDistanceMeasurementAreaX = millis();
			sprintf(errorHandler.msg, "!03,DistAreax M: %ld encCounts %ld\r\n", drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}


	return drivenDistance;
}






// Fährt vor und zurück nach Position. Test von TPositionControl
void TMotorInterface::testPosForwardStopBackward()
{

	static unsigned long  nextTime = 0;
	unsigned long now = 0;
	static int state = 0;

	now = millis();
	if (now < nextTime) return;
	nextTime = millis() + 100;  //=Ta


								//debug << " now " << now << " lasttime " << lasttime <<  endl;;


	switch (state) {
	case 0:
		errorHandler.setInfo(F("0\r"));

		rotateAngle(180, 50);
		state = 1;

		break;

	case 1:
		errorHandler.setInfo(F("1\r"));
		if (isPositionReached()) {
			rotateAngle(-180, 50);
			state = 2;
		}
		break;
	case 2:
		errorHandler.setInfo(F("2\r"));
		if (isPositionReached()) {
			rotateAngle(180, 50);
			state = 1;
		}
		break;

	default:
		break;
	}

}


