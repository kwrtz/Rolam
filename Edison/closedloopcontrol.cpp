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
#include "closedloopcontrol.h"
#include "global.h"
#include "hardware.h"
#include "errorhandler.h"
#include "tables.h"


//#define DRIVEMOTOR_TEST 1   // use this to activate test routines in the bottem of this script





void TClosedLoopControlThread::setup(uint8_t motorNumber, CRotaryEncoder  *enc)    // Motor 1 oder 2
{

	lasttTimeEncoderRead = 0;
	lasttTimeSpeedShown = 0;

	Setpoint = 0.0f;
	current_speed = 0.0f;

	Output = 0.0f; //
	flagShowSpeed = false;

	enableDefaultRamping();

	sollSpeed = 0; // Soll speed wird von setSpeed gesetzt und somit vom user gesetzt
				   // Motordriver initialisieren
				   //SerDueMot.begin(19200);
				   //motordriver.autobaud();

				   // Welcher motor wird am Sabertooth angesteuert 1 oder 2
	motorNo = motorNumber;
	myEncoder = enc;

	myPID.setup(&current_speed, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
	myPID.SetOutputLimits(-127, 127);
	myPID.SetMode(AUTOMATIC);

	current_speed_mph = 0;
	lastTickCounter = myEncoder->getTickCounter();
	SetState(STM_STOP);
}


/*********************************************************/
// Motor FSM ausführen
/*********************************************************/
void TClosedLoopControlThread::run()
{
	// Wird alle 20ms aufgerufen
	runned();
	readEncoder();
//	LoopFSM();
}

/*********************************************************/
// Motor Zustände Do State
/*********************************************************/
void TClosedLoopControlThread::UpdateState(EMotorState t)
{
	switch (t) {


	case STM_RUN:
		//debug.serial.println("STM_RUN\r\n");
		rampSetpoint(sollSpeed);
		myPID.Compute();
		//debug->printf("Setpoint: %f Output: %f\r\n", Setpoint, Output);

		// Agilität verbessern
		if (sollSpeed>0) {
			if (fabs(Output)<7) {
				Output = 7;
			}
		}
		else if (sollSpeed<0) {
			if (fabs(Output)<7) {
				Output = -7;
			}
		}
		else if (sollSpeed == 0) {
			if (fabs(Output)<15) {
				Output = 0;
			}
		}

		motordriver.motor(motorNo, Output);

		// When motor stalls, Sabertooth handle this situatin and current.
		// I only have here to realize that the robot is not driving.
		// Check motorstall alle 5 Sekunden. Dann sollten sich Encoder auf jeden Fall bewegt haben.
		if (millis() - timeLastCheckMotorStall > 5000) {
			timeLastCheckMotorStall = millis();
			unsigned long buff = myEncoder->getAbsTicksCounter();
			if ((buff - lastAbsTickCounter) < 100) {  // Ca. 16° Radbewegung
				errorHandler.setError(F("clc STM_RUN  Motor Stall\r\n"));
				motordriver.motor(motorNo, 0);
			}
			lastAbsTickCounter = buff;
		}

		break;


	case STM_STOP_REQUEST:
		//debug.serial.println("STM_STOP_REQUEST");
		rampSetpoint(0);
		myPID.Compute();
		//debug->printf("SetpointStopR: %f Output: %f  fabsOutput: %f\r\n", Setpoint, Output, fabs(Output));
		motordriver.motor(motorNo, Output);


		if (fabs(Output) < 15) { //Bei Output 15 schon stop damit robbi nicht zu träge ist
								 //debug->printf("SetpointStop<1: %f Output: %f\r\n", Setpoint, Output);
			motordriver.motor(motorNo, 0);
			SetState(STM_STOP_ROLLOUT);
		}
		break;

	case STM_STOP_ROLLOUT:
		//debug.serial.println("STM_STOP_ROLLOUT");
		motordriver.motor(motorNo, 0);
		if (fabs(current_speed) < 100) { // Wait until motor stopped. Bei 100ticks/s schon OK damit robbi nicht zu träge ist aber immer noch sanft abbremst
			SetState(STM_STOP);
		}

		break;

	case STM_STOP:
		//debug.serial.println("STM_STOP");
		motordriver.motor(motorNo, 0);
		break;

	default:
		//TODO invalid state - reset, perhaps?
		break;
	}

};

/*********************************************************/
// Motor Zustände Entry State
/*********************************************************/
void TClosedLoopControlThread::BeginState(EMotorState t)
{
	switch (t) {
	case STM_RUN:
		//debug.serial.println("STM_RUN BeginState \r\n");
		myPID.Initialize();
		lastTickCounter = myEncoder->getTickCounter();
		lastAbsTickCounter = myEncoder->getAbsTicksCounter();
		//myEncoder->resetTickCounter();
		lasttTimeEncoderRead = millis();
		timeLastCheckMotorStall = millis();
		break;

	default:
		//TODO invalid state - reset, perhaps?
		break;
	}


};

/*********************************************************/
// Motor Zustände Exit State
/*********************************************************/
void TClosedLoopControlThread::EndState(EMotorState t)
{
	switch (t) {

	case STM_STOP_REQUEST:
		//debug->puts("STM_STOP_REQUEST EndState\r\n");
		Output = 0;
		Setpoint = 0;
		break;

	case STM_STOP:
		//debug->puts("STM_STOP EndState\r\n");
		Output = 0;
		Setpoint = 0;
		myPID.Initialize();
		lastTickCounter = myEncoder->getTickCounter();
		//myEncoder->resetTickCounter();
		break;

	default:
		//TODO invalid state - reset, perhaps?
		break;
	}
};

// ---------------------------------------------------------
// speed is -100% to +100%. This function calculates the ticks pro sek for the given percentage.
// ---------------------------------------------------------
void TClosedLoopControlThread::setSpeed(long  speed)
{
	long y;
	y = 0;
	if (speed != 0) {
		y = mapl(speed, -100, 100, -1 * GETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED), GETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED));
	}

	sollSpeed = y;

	if (GetState() != STM_RUN) {
		SetState(STM_RUN);
	}
}

// ---------------------------------------------------------
// speed in ticks per seconds. used by position control
// ---------------------------------------------------------
void TClosedLoopControlThread::setSpeedTPS(long  speed)
{

	if (speed < -1 * GETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED)) {
		speed = -1 * GETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED);
	}
	else if (speed > GETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED)) {
		speed = GETTF(TF_MAX_ENCTICKS_PER_SEC_AT_FULL_SPEED);
	}

	sollSpeed = speed;

	if (GetState() != STM_RUN)
		SetState(STM_RUN);
}


// Calculate current speed
void TClosedLoopControlThread::readEncoder()
{
	long encTickCounter, buff;
	unsigned long nowTime;

	// Werte aus Encoder holen und encoder wieder auf 0 setzen
	buff = myEncoder->getTickCounter();
	encTickCounter = buff - lastTickCounter;
	lastTickCounter = buff;
	//myEncoder->resetTickCounter();

	nowTime = millis();
	unsigned long delta = nowTime - lasttTimeEncoderRead;
	double deltaTime = delta;//time between most recent encoder ticks

	if (delta != 0) {
		//current_speed = encTickCounter * 1000.0f / deltaTime; // current_speed in ticks pro sekunde
		current_speed = 0.625f * current_speed + 0.375f * (encTickCounter * 1000.0f / deltaTime);   // current_speed in ticks pro sekunde
	}

	lasttTimeEncoderRead = nowTime;
	// TICKSPERM


	//debug->printf("deltaTicks: %d\r\n", encTickCounter);


	current_speed_mph = (current_speed * 3600 * GETTF(TF_RADUMFANG_CM)) / (100.0f * GETTF(TF_ENCTICKSPERROTATION));  // current_speed in m pro h

																													 //flagShowSpeed= true;
	if (flagShowSpeed) {
		//current_speed_mph =  current_speed  * 3600  /   (100.0f * GETTF(TF_ENCTICKSPERROTATION) / GETTF(TF_RADUMFANG_CM));  // current_speed in m pro h
		if (millis() - lasttTimeSpeedShown > 1000) {
			lasttTimeSpeedShown = millis();
			sprintf(errorHandler.msg, "!03,motor %i c/s: %f  m/h: %f deltaTicks: %lu deltaTime: %lu Setpoint: %f Output: %f\r\n", motorNo, current_speed, current_speed_mph, encTickCounter, delta, Setpoint, Output);
			errorHandler.setInfo();
		}
	}


}



void TClosedLoopControlThread::enableDefaultRamping()
{
	//debug->printf("enableDefaultRamping()\r\n" );
	useRamp = true;
	Kp = 0.2f;
	Ki = 0.5f;
	Kd = 0.0f;
	acceleration = 16;// Konstante für beschleunigung
	deceleration = 16;//Konstante für langsamer werden
}
void TClosedLoopControlThread::enablePerTrackRamping()
{
	//useRamp = false;
	//debug->printf("enablePerTrackRamping()\r\n" );
	useRamp = true;
	Kp = 0.2f;
	Ki = 0.5f;
	Kd = 0.0f;
	acceleration = 40;// Konstante für beschleunigung
	deceleration = 40;//Konstante für langsamer werden
}

void TClosedLoopControlThread::enableFastStopRamping()
{
	//useRamp = false;
	//debug->printf("enableFastStopRamping()\r\n");
	useRamp = true;
	Kp = 0.2f;
	Ki = 0.5f;
	Kd = 0.0f;
	acceleration = 125;// Konstante für beschleunigung
	deceleration = 125;//Konstante für langsamer werden
}


/*+ ---------------------------------------------------------
Called at the loop rate to add "velocity" to the set point thus
effecting a motion.
Velocity is ramped up and down by "acceleration/deceleration"
-*/
void TClosedLoopControlThread::rampSetpoint(double  _sollSpeed)
{
	if (useRamp) {

		if (Setpoint < _sollSpeed) {
			Setpoint += acceleration;

			if (_sollSpeed > 0 && Setpoint > 0 && Setpoint < 20) { // Wenn von speed 0 gestartet wird Setpoint bei 20ticks/s starten um weniger träge zu sein. Speed von 5% =  _sollspeed: 57.000000
				Setpoint = 20;
				//debug->puts("Setpoint = 20;\r\n");
			}
			if (Setpoint > _sollSpeed) {
				Setpoint = _sollSpeed;
			}

		}
		else if (Setpoint > _sollSpeed) {
			Setpoint -= deceleration;
			if (_sollSpeed < 0 && Setpoint < 0 && Setpoint > -20) { // Wenn von speed 0 gestartet wird Setpoint bei -20 anfangen
																	//debug->printf("Setpoint %f, _sollspeed: %f\r\n", Setpoint, _sollSpeed);
				Setpoint = -20;
				//debug->puts("Setpoint = -20;\r\n");
			}
			if (Setpoint < _sollSpeed) {
				Setpoint = _sollSpeed;
			}

		}

	}//  if(useRamp) {
	else {
		Setpoint = _sollSpeed;
	}
	//debug << "_rampSetpoint( _sollSpeed)" << " _sollSpeed: " << _sollSpeed << "rampedSetpoint:  " << rampedSetpoint << " Setpoint: " << Setpoint << endl;
}


void TClosedLoopControlThread::hardStop()
{
	motordriver.motor(motorNo, 0);
	sollSpeed = 0;
	if (GetState() != STM_STOP_ROLLOUT && GetState() != STM_STOP) { // Verhindern, dass falls bereits in STM_STOP dieser nicht wieder in STM_STOP_ROLLOUT gesetzt wird.
																	// Dann kann es sein, dass bei Aufruf in loop nie in STOP kommmt, da immer STM_STOP_ROLLOUT gesetzt wird.
		SetState(STM_STOP_ROLLOUT);
		motordriver.motor(motorNo, 0);
	}
}

void TClosedLoopControlThread::stop()
{
	sollSpeed = 0;


	if (GetState() != STM_STOP_REQUEST && GetState() != STM_STOP_ROLLOUT && GetState() != STM_STOP) { // Verhindern, dass falls bereits in STM_STOP dieser nicht wieder in STM_STOP_REQUEST gesetzt wird.
																									  // Dann kann es sein, dass bei Aufruf in einem loop nie in STOP kommmt, da immer STM_STOP_ROLLOUT gesetzt wird.
		SetState(STM_STOP_REQUEST);
	}
}


bool TClosedLoopControlThread::isStopped()
{
	return (GetState() == STM_STOP);
}

// ---------------------------------------------------------
// Sabnertooth settings only needed one time for configuration of sabertooth
// ---------------------------------------------------------

void TClosedLoopControlThread::setRamp()
{
	//The Sabertooth remembers this command between restarts AND in all modes.
	//motordriver.setRamping(1); // 14 ca 4 sek   18 ca. 2sek
}

void TClosedLoopControlThread::setBaudRate()
{
	//The Sabertooth remembers this command between restarts AND in all modes.
	// motordriver.setBaudRate(19200);
}

void TClosedLoopControlThread::controlDirect(int speed)
{

	if (speed < -127) speed = -127;
	else if (speed > 127) speed = 127;

	motordriver.motor(motorNo, speed);

}

/*

// FFarray is the array of expected motor velocities (units: sensorValue per second) that correspond with the duty cycle in increments of 20
static double FFarray[11] = { -228, -182, -136, -91, -46, 0, 46, 91, 136, 182, 228 };

// FFdutycycle is the array of duty cycles in increments of 20. This array is constant.
static int FFdutycycle[11] = { -100,-80, -60,-40,-20, 0,20, 40, 60, 80,100 };

// The function getFF() simply converts velocityTarget from SensorValue/second to the estimated matching duty cycle using an array of pre-defined values and linear interpolation between those values.
// output = (Kp * velocityError) + (Kf * getFF(velocityTarget)) + (Ki * integral);
double TClosedLoopControlThread::getFF(double feedforward)
{
for (int i = 0; i < 10; i++)
{
// If feedforward is between two points on the characteristic curve,
if ((feedforward >= FFarray[i]) && (feedforward <= FFarray[i + 1]))
{
//  return (int)interpolate(FFarray[i], FFdutycycle[i], FFarray[i + 1], FFdutycycle[i + 1], feedforward);
}
}

// If feedforward is greater than the maximum value of the characteristic curve,
if (feedforward > FFarray[10])
return 100; // return the maximum duty cycle
// If feedforward is less than the minimum value of the characteristic curve,
else if (feedforward < FFarray[0])
return -100; // return the minimum duty cycle
// Else, something is wrong with the characteristic curve and the motor should be stopped
else
return 0;
}

*/





/********************************************************************************************************
TESTFUNKTIONEN
Da static variabeln enthalten nur ausführen wenn nur ein Objekt der Klasse TDriveMotor testen!!!
*******************************************************************************************************/




#ifdef  DRIVEMOTOR_TEST


void TClosedLoopControlThread::testEncoder()
{
	static unsigned long  nextTime = 0;
	//   static unsigned long lasttime = 0;
	unsigned long now = 0;
	//   static int speed = 0;


	now = millis();

	if (now < nextTime) return;
	nextTime = millis() + 100;  //=Ta

								//  debug->printf("micro32: %lu millis: %lu\r\n", _microseconds32, now);
								//debug->printf("millis: %lu\r\n", now);



	controlDirect(20);

	int encTickCounter = myEncoder->getTickCounter();
	sprintf(errorHandler.msg, "!03,deltaTicks: %d\r\n", encTickCounter);
	errorHandler.setInfo();

	/*
	if (now - lasttime > 1000) {
	speed++;
	lasttime = now;
	}
	if (speed > 127) speed = 0;

	motordriver.motor(motorNo, speed);
	Output = speed;
	*/

	//debug->puts("A\r");

}


void TClosedLoopControlThread::testMotors()
{
	int speed;

	// Ramp from -127 to 127 (full reverse to full forward), waiting 20 ms (1/50th of a second) per value.
	for (speed = -127; speed <= 127; speed++) {
		motordriver.motor(motorNo, speed);
		delay(20);
	}

	// Now go back the way we came.
	for (speed = 127; speed >= -127; speed--) {
		motordriver.motor(motorNo, speed);
		delay(20);
	}

}


void TClosedLoopControlThread::testForwardStopBackward()
{

	static unsigned long  nextTime = 0;
	static unsigned long lasttime = 3000;
	unsigned long now = 0;
	int state = 0;

	now = millis();
	if (now < nextTime) return;
	nextTime = millis() + 100;  //=Ta

	readEncoder();
	//debug << " now " << now << " lasttime " << lasttime <<  endl;;


	switch (state) {
	case 0:
		//forward();
		if ((now - lasttime) > 3000) {
			state = 2;
			lasttime = now;
			stop();
		}

		break;

	case 1:
		if (isStopped()) {
			setSpeed(50);
			backward();
			state = 2;
		}

		break;

	case 2:
		if ((now - lasttime) > 3000) {
			state = 3;
			lasttime = now;
			stop();
		}
		break;

	case 3:
		if (isStopped()) {
			setSpeed(50);
			forward();
			state = 1;
		}
		break;

	default:
		break;
	}

}

void TClosedLoopControlThread::testReadEncoder()
{

	static unsigned long lasttime = 0;
	unsigned long now = 0;
	static int speed = 0;


	now = millis();

	if (now - lasttime > 1000) {
		speed++;
		lasttime = now;
	}
	if (speed > 127) speed = 0;


	motordriver.motor(motorNo, speed);
	debug.print("speed:  ");
	debug.println(speed);

	// Alle 50ms Encoder aktualisieren
	static unsigned long  nextTimeMotorControl = 0;
	if (millis() < nextTimeMotorControl) return;
	nextTimeMotorControl = millis() + 50;  //=Ta

	readEncoder();

}

void TClosedLoopControlThread::testEncoder()
{

	static unsigned long lasttime = 0;
	unsigned long now = 0;
	static int speed = 0;


	now = millis();

	if (now - lasttime > 1000) {
		speed++;
		lasttime = now;
	}
	if (speed > 127) speed = 0;

	motordriver.motor(motorNo, speed);

	if (myEncoder->interruptExecuted() == true) {

		myEncoder->deleteFlagIntExecuted();
		debug.print("speed:  ");
		debug.print(speed);
		debug.print("   getPulseTime ml:  ");
		debug.print(myEncoder->getPulseTime());
		debug.print("   getPosition ml:  ");
		debug.println(myEncoder->getTickCounter());

	}
}

#endif



