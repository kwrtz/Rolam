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
#include "global.h"
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
	posKp = 2;
	sollSpeed = 0; // Soll speed wird von setSpeed gesetzt und somit vom user gesetzt
	sollDistance = 0;

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
	// Alle 100ms Encoder aktualisieren
	LoopFSM();
}


/*********************************************************/
// Position Zustände Do State
/*********************************************************/
void TPositionControl::UpdateState(EPosState t)
{

	long   speed, positionError;
	long positionCounter;



	switch (t) {
	case STP_START_ANGLE:
		//Serial.println("STP_START_ANGLE ");
		//myEncoder->resetPositionCounter();
		startPositionCounter = myEncoder->getTickCounter();
		sollDistance = getCountsForDegree(sollAngle);

		//sprintf(errorHandler.msg,"!03,sollDistance: %ld lastPositionCounter %ld\r\n",sollDistance, lastPositionCounter);
		//errorHandler.setInfo();
		if (sollAngle < 0)
			sollDistance -= 40; //40 counts dazuzählen, da unten bei if (positionError < 28) gestoppt wird
		else if (sollAngle > 0)
			sollDistance += 40; //40 counts dazuzählen, da unten bei if (positionError < 28) gestoppt wird
		SetState(STP_DRIVETOPOSITION);
		break;


	case STP_START_CM:
		//Serial.println("STP_START_ANGLE ");
		//myEncoder->resetPositionCounter();
		startPositionCounter = myEncoder->getTickCounter();
		sollDistance = getCountsForCM(sollCM);

		//sprintf(errorHandler.msg,"!03,sollDistance: %ld lastPositionCounter %ld\r\n",sollDistance, lastPositionCounter);
		//errorHandler.setInfo();
		if (sollCM < 0)
			sollDistance -= 40; //40 counts dazuzählen, da unten bei if (positionError < 28) gestoppt wird
		else if (sollCM > 0)
			sollDistance += 40; //40 counts dazuzählen, da unten bei if (positionError < 28) gestoppt wird
		SetState(STP_DRIVETOPOSITION);
		break;



	case STP_DRIVETOPOSITION:

		positionCounter = myEncoder->getTickCounter() - startPositionCounter;

		//sprintf(errorHandler.msg,"!03,buff: %ld positionCounter %ld\r\n",buff, positionCounter);
		//errorHandler.setInfo();

		if (abs(sollDistance) > abs(positionCounter)) { // Wurde sollDistanz bereits überfahren?

			positionError = sollDistance - positionCounter;
			speed = posKp *positionError;

			//debug->printf("1. positionError: %ld speed: %ld sollSpeed: %ld\r\n",   positionError, speed, sollSpeed);

			if (abs(speed) > abs(sollSpeed))
				speed = sollSpeed;

			//debug->printf("2. positionError: %ld speed: %ld\r\n",   positionError, speed);

			motor->setSpeedTPS(speed);

		}
		else { // Falls sollposition bereits überfahren
			positionError = 0;
		}

		// If the error is within the specified deadband, and the motor is moving slowly enough. Da speed proportional zu positionError ist, muss nur speed abgefragt werden.
		if (abs(positionError) < 60) { // && positionError < 60) { // 28
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
			//debug->printf("sollDistance: %ld gefahrene Counts: %ld gefahrener Winkel: %ld\r\n", getCountsForDegree(sollAngle), myEncoder->getPositionCounter(), getDegreeForCounts(myEncoder->getPositionCounter()));
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
	long y;
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
	long y;
	y = x * GETTF(TF_ENCTICKSPERROTATION) / 360.0f;
	return y;
}

long TPositionControl::getDegreeForCounts(float x)
{
	long y;
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

// ---------------------------------------------------------
// speed is 0%-100%. This function calulates the ticks pro sek for the given percentage.
// ---------------------------------------------------------
void TPositionControl::setSpeed(long  speed)
{
	unsigned long y;
	y = 0;
	if (speed != 0)
		y = mapl(speed, -100, 100, -1 * GETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED), GETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED));

	sollSpeed = y;
}


void TPositionControl::rotateAngle(float _angle, long _speed)
{
	sollAngle = _angle;

	if (_angle < 0)
		setSpeed(-1 * abs(_speed));
	else
		setSpeed(abs(_speed));


	SetState(STP_START_ANGLE);
}

void TPositionControl::rotateCM(float _cm, long _speed)
{
	sollCM = _cm;

	if (_cm < 0)
		setSpeed(-1 * abs(_speed));
	else
		setSpeed(abs(_speed));


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
	sollSpeed = 0;
	SetState(STP_STOP);
}

void TPositionControl::SetPosKp(double Kp)
{

	posKp = Kp;

}











