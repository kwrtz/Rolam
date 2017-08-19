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

#include <math.h>
#include "positioncontrol.h"
#include "helpers.h"
#include "hardware.h"
#include "tables.h"
#include "errorhandler.h"


//static const double  ENCTICKSPERROTATION = 2120.0f;  // Anzahl encoder flanken pro umdrehung - positive und negative flanken.
//static const double  RADUMFANG = 78.54f;             // Radumfang in cm


/**********************************************************************************/
/**********************************************************************************/
// Position control
/**********************************************************************************/
/**********************************************************************************/

void TPositionControl::setup(TClosedLoopControlThread *_motor, CRotaryEncoder *enc)
{

	flagShowResults = false;
	sollSpeedPercentage = 0; // Soll speed wird von setSpeed gesetzt und somit vom user gesetzt
	sollPositionCM = 0;

	posKp = 2;
	stopCmBeforeTarget = 5;  // Must be positive and >0
	addCmToTargetPosition = 4;

	motor = _motor;
	myEncoder = enc;
	SetState(STP_STOP);
}


/*********************************************************/
// Pos FSM ausführen
/*********************************************************/
void TPositionControl::run()
{
	runned();
	LoopFSM();
}


/*********************************************************/
// Position Zustände Do State
/*********************************************************/
void TPositionControl::UpdateState(EPosState t)
{

	float positionErrorCM, istPositionCM;
	long  speed;



	switch (t) {

	case STP_START_CM:
		//Serial.println("STP_START_ANGLE ");
		//myEncoder->resetPositionCounter();
		startPositionCM = getCMForCounts(myEncoder->getTickCounter());

		inputPositionCM = sollPositionCM; // Save user input position in order sollPositionCM will be changed below

		if (sollPositionCM < 0)
			sollPositionCM -= addCmToTargetPosition; //cm dazuzählen, da unten bei if (positionError < 28) gestoppt wird
		else if (sollPositionCM > 0)
			sollPositionCM += addCmToTargetPosition; //cm dazuzählen, da unten bei if (positionError < 28) gestoppt wird
		SetState(STP_DRIVETOPOSITION);

		if (flagShowResults) {
			errorHandler.setInfoNoLog(F("!03,PC motor %i soll changed from: %fcm to: %fcm\r\n"), motor->motorNo, inputPositionCM, sollPositionCM);
		}
		break;



	case STP_DRIVETOPOSITION:

		istPositionCM = getCMForCounts(myEncoder->getTickCounter()) - startPositionCM;

		//sprintf(errorHandler.msg,"!03,buff: %ld positionCounter %ld\r\n",buff, positionCounter);
		//errorHandler.setInfo();

		if (abs(sollPositionCM) > abs(istPositionCM)) { // Wurde sollDistanz noch nicht erreicht?

			positionErrorCM = sollPositionCM - istPositionCM;
			speed = posKp *positionErrorCM;

			//debug->printf("1. positionError: %ld speed: %ld sollSpeed: %ld\r\n",   positionError, speed, sollSpeed);

			if (abs(speed) > abs(sollSpeedPercentage))
				speed = sollSpeedPercentage;

			//debug->printf("2. positionError: %ld speed: %ld\r\n",   positionError, speed);

			motor->setSpeed(speed);

		}
		else { // Falls sollposition bereits überfahren
			positionErrorCM = 0;
		}

		if (abs(positionErrorCM) < stopCmBeforeTarget) {
			//debug->printf("abs(speed):  %ld positionError: %ld \r\n",  abs(speed), positionError);
			speed = 0;
			SetState(STP_TARGETREACHED); // Solldistanz erreicht
			motor->stop();
		}

		break;
		/*
		case STP_STOPPOSITIONING:
		speed = 0;
		motor->stop();
		SetState(STP_TARGETREACHED); // Solldistanz erreicht
		break;
		*/
	case STP_TARGETREACHED:
		//Serial.println("STP_TARGETREACHED");
		if (motor->isStopped()) {

			if (flagShowResults) {
				istPositionCM = getCMForCounts(myEncoder->getTickCounter()) - startPositionCM;
				positionErrorCM = inputPositionCM - istPositionCM;
				errorHandler.setInfoNoLog(F("!03,PC motor %i soll: %f  ist: %f error: %f\r\n"), motor->motorNo, inputPositionCM, istPositionCM, positionErrorCM);
			}

			SetState(STP_STOP);
		}
		break;

	case STP_STOP:
		//Serial.println("STP_STOP");
		break;

	default:
		//TODO invalid state - reset, perhaps?
		break;
	}

};



// ---------------------------------------------------------
// Umrechnungs Routinen
// ---------------------------------------------------------

long TPositionControl::getCountsForCM(float x)
{
	float y;
	y = (x * GETTF(TF_ENCTICKSPERROTATION)) / GETTF(TF_RADUMFANG_CM);
	return y;
}

float TPositionControl::getCMForCounts(float x)
{
	float y;
	y = (x * GETTF(TF_RADUMFANG_CM)) / GETTF(TF_ENCTICKSPERROTATION);
	return y;
}

long TPositionControl::getCountsForDegree(float x)
{
	float y;
	y = x * GETTF(TF_ENCTICKSPERROTATION) / 360.0f;
	return y;
}

float TPositionControl::getDegreeForCounts(float x)
{
	float y;
	y = x * 360.0f / GETTF(TF_ENCTICKSPERROTATION);
	return y;
}

// ---------------------------------------------------------
// Wurde Sollposition erreicht? Wenn ja werden rotateForward/Backward durch positionReached gelatched
// Dieser Zustand muss vor beginn einer neuen Wegmessung  mit resetPositionReached() zurückgesetzt werden.
// ---------------------------------------------------------
bool  TPositionControl::isPositionReached()
{
	return (GetState() == STP_STOP);
}



void TPositionControl::rotateAngle(float _angle, long _speedPercentage)
{

	long counts = getCountsForDegree(_angle);
	sollPositionCM = getCMForCounts(counts);
	rotateCM(sollPositionCM, _speedPercentage);


}

void TPositionControl::rotateCM(float _cm, long _speedPercentage)
{
	sollPositionCM = _cm;

	if (_cm < 0)
		sollSpeedPercentage = (-1 * abs(_speedPercentage));
	else
		sollSpeedPercentage = (abs(_speedPercentage));


	SetState(STP_START_CM);
}

/*
void TPositionControl::stopPositioning()
{
if( GetState() != STP_STOPPOSITIONING && GetState() != STP_TARGETREACHED && GetState() != STP_STOP) {
SetState(STP_STOPPOSITIONING);
}
}
*/

void TPositionControl::reset()  // Motoren müssen vorher angehalten werden
{
	sollSpeedPercentage = 0;
	SetState(STP_STOP);
}

void TPositionControl::showConfig()
{
	errorHandler.setInfoNoLog(F("!03,PC Config MotorNo: %i\r\n"), motor->motorNo);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
	errorHandler.setInfoNoLog(F("!03,posKp: %f \r\n"), posKp);
	errorHandler.setInfoNoLog(F("!03,stopCmBeforeTarget %f\r\n"), stopCmBeforeTarget);
	errorHandler.setInfoNoLog(F("!03,addCmToTargetPosition %f\r\n"), addCmToTargetPosition);
}










