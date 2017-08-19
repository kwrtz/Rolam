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
#include "helpers.h"
#include "hardware.h"
#include "errorhandler.h"
#include "tables.h"



void TClosedLoopControlThread::setup(uint8_t motorNumber, CRotaryEncoder  *enc)    // Motor 1 oder 2
{

	lastTimeEncoderRead = 0;
	lastTimeSpeedShown = 0;

	setpointRPM = 0.0f;
	current_speedRPM = 0.0f;
	Output = 0.0f; //
	feedforwardRPM = 0.0f;

	flagShowSpeed = false;
	flagShowEncoder = false;
	flagMotorStepSpeed = false;
	flagMotorFSB = false;
	flagControldirect = false;

	lastrunTest = 0;

	sollSpeedRPM = 0; // Soll speed wird von setSpeed gesetzt und somit vom user gesetzt
				   // Motordriver initialisieren
				   //SerDueMot.begin(19200);
				   //motordriver.autobaud();

				   // Welcher motor wird am Sabertooth angesteuert 1 oder 2
	motorNo = motorNumber;
	myEncoder = enc;
													   //Kp, Ki, Kd Will be set in enableDefaultRamping();
	myPID.setup(&current_speedRPM, &Output, &setpointRPM, 0, 0, 0, DIRECT);
	enableDefaultRamping();
	myPID.SetOutputLimits(-127, 127);
	myPID.SetMode(AUTOMATIC);

	current_speed_mph = 0;
	lastTickCounter = myEncoder->getTickCounter();
	SetState(STM_STOP);
}

void TClosedLoopControlThread::enableDefaultRamping()
{
	//debug->printf("enableDefaultRamping()\r\n" );
	myPID.SetTunings(2,3,0);
	useRamp = true;
	rampAccRPM = 1.7; // Konstante für beschleunigung
	rampDecRPM = 1.7; //Konstante für langsamer werden
	deadbandRPM = 2;
	setOutputZeroAtRPm = 3;
	stopThresholeAtRpm = 1;
}

void TClosedLoopControlThread::enablePerTrackRamping()
{
	//useRamp = false;
	//debug->printf("enablePerTrackRamping()\r\n" );
	myPID.SetTunings(2, 3, 0);
	useRamp = true;
	rampAccRPM = 4; // Konstante für beschleunigung
	rampDecRPM = 4; //Konstante für langsamer werden
	deadbandRPM = 2;
	setOutputZeroAtRPm = 3;
	stopThresholeAtRpm = 1;
}

void TClosedLoopControlThread::enableFastStopRamping()
{
	//useRamp = false;
	//debug->printf("enableFastStopRamping()\r\n");
	myPID.SetTunings(2, 3, 0);
	useRamp = true;
	rampAccRPM = 10;// Konstante für beschleunigung
	rampDecRPM = 10;//Konstante für langsamer werden
	deadbandRPM = 2;
	setOutputZeroAtRPm = 3;
	stopThresholeAtRpm = 1;
}
/*********************************************************/
// Motor FSM ausführen
/*********************************************************/
void TClosedLoopControlThread::run()
{
	// Wird alle 100ms aufgerufen
	runned();

	// Test functions start
	if (flagShowEncoder) {
		showEncoder();
	}

	if (flagControldirect) {
		return;
	} 
	else if (flagMotorStepSpeed) {
		motorStepSpeed();
	}
	else if (flagMotorFSB) {
		testForwardStopBackward();
	}
	// Test functions end

	readEncoder();
	LoopFSM();
}

/*********************************************************/
// Motor Zustände Do State
/*********************************************************/
void TClosedLoopControlThread::UpdateState(EMotorState t)
{

	switch (t) {

	case STM_RUN:
		//debug.serial.println("STM_RUN\r\n");
		rampSetpoint(sollSpeedRPM);
		myPID.Compute();
		feedforwardRPM = setpointRPM * 100.0 / GETTF(TF_MAX_WHEEL_RPM);  // feed forward should be lower than the needed output. therefore *90 instead of *127 which is maximum speed

		// Agilität verbessern
/*
		 if (sollSpeedRPM>0) {
			if (fabs(Output)<7) {
				Output = 7;
			}
		}
		else if (sollSpeedRPM<0) {
			if (fabs(Output)<7) {
				Output = -7;
			}
		}
*/
		if (fabs(sollSpeedRPM) < 0.0001) {
			if (fabs(current_speedRPM) < setOutputZeroAtRPm) {
				Output = 0;
				feedforwardRPM = 0;
			}
		}

		motorDriver.motor(motorNo, feedforwardRPM + Output);

		if (flagShowSpeed) {
			errorHandler.setInfoNoLog(F("sp: %f ff: %f out: %f ff+out: %f\r\n"), setpointRPM, feedforwardRPM, Output, feedforwardRPM + Output);

   	   }

		// When motor stalls, Sabertooth handle this situatin and current.
		// I only have here to realize that the robot is not driving.
		// Check motorstall alle 5 Sekunden. Dann sollten sich Encoder auf jeden Fall bewegt haben.
/*		if (millis() - timeLastCheckMotorStall > 5000) {
			timeLastCheckMotorStall = millis();
			unsigned long buff = myEncoder->getAbsTicksCounter();
			if ((buff - lastAbsTickCounter) < 100) {  // Ca. 16° Radbewegung
				errorHandler.setError(F("clc STM_RUN  Motor Stall\r\n"));
				motorDriver.motor(motorNo, 0);
			}
			lastAbsTickCounter = buff;
		}
		*/

		break;


	case STM_STOP_REQUEST:
		//debug.serial.println("STM_STOP_REQUEST");
		rampSetpoint(0);
		feedforwardRPM = setpointRPM * 100 / GETTF(TF_MAX_WHEEL_RPM);
		myPID.Compute();
		//debug->printf("SetpointStopR: %f Output: %f  fabsOutput: %f\r\n", setpointRPM, Output, fabs(Output));
		motorDriver.motor(motorNo, feedforwardRPM + Output);

		if (flagShowSpeed) {
			errorHandler.setInfoNoLog(F("sp: %f ff: %f out: %f ff+out: %f\r\n"), setpointRPM, feedforwardRPM, Output, feedforwardRPM + Output);
		}

		if (fabs(current_speedRPM) < setOutputZeroAtRPm) { //Bei 4RPM schon stop damit robbi nicht zu träge ist
								 //debug->printf("SetpointStop<1: %f Output: %f\r\n", setpointRPM, Output);
			motorDriver.motor(motorNo, 0);
			SetState(STM_STOP_ROLLOUT);
		}
		break;

	case STM_STOP_ROLLOUT:
		//debug.serial.println("STM_STOP_ROLLOUT");
		motorDriver.motor(motorNo, 0);
		if (flagShowSpeed) {
			errorHandler.setInfoNoLog(F("sp: %f ff: %f out: %f ff+out: %f\r\n"), setpointRPM, 0.0, 0.0, 0.0);
		}

		if (fabs(current_speedRPM) < stopThresholeAtRpm) { // Wait until motor stopped. Bei 1 Umdrehungen/minute schon OK damit robbi nicht zu träge ist aber immer noch sanft abbremst
			SetState(STM_STOP);
		}

		break;

	case STM_STOP:
		//debug.serial.println("STM_STOP");
		motorDriver.motor(motorNo, 0);
		if (flagShowSpeed) {
			errorHandler.setInfoNoLog(F("sp: %f ff: %f out: %f ff+out: %f\r\n"), setpointRPM, 0.0, 0.0, 0.0);
		}

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
		lastTimeEncoderRead = millis();
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
		setpointRPM = 0;
		break;

	case STM_STOP:
		//debug->puts("STM_STOP EndState\r\n");
		Output = 0;
		setpointRPM = 0;
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
// speed is -100% to +100%. This function calculates rpm for the given percentage.
// ---------------------------------------------------------
void TClosedLoopControlThread::setSpeed(long  speedPercentage)
{
	if (speedPercentage > 100){
		speedPercentage = 100;
	}
	else if(speedPercentage < -100){
		speedPercentage = -100;
	}
	sollSpeedRPM = (GETTF(TF_MAX_WHEEL_RPM) * (double)speedPercentage) / 100.0;

	if (GetState() != STM_RUN) {
		SetState(STM_RUN);
	}
}



// Calculate current speed
void TClosedLoopControlThread::readEncoder()
{
	long encTickCounter, buff;
	unsigned long nowTime;

	// Delta Encoder ermitteln
	buff = myEncoder->getTickCounter();
	encTickCounter = buff - lastTickCounter;
	lastTickCounter = buff;

	// Delta Time ermitteln 
	nowTime = millis();
	unsigned long delta = nowTime - lastTimeEncoderRead;
	lastTimeEncoderRead = nowTime;
	double deltaTimeSec = (double)delta/1000.0;//time between most recent encoder ticks
	double ticksPerRevolution = GETTF(TF_ENCTICKSPERROTATION);

	if (delta != 0) {
		current_speedRPM = 60.0 * ( (double)encTickCounter / ticksPerRevolution) / deltaTimeSec;
	}


	//debug->printf("deltaTicks: %d\r\n", encTickCounter);


	current_speed_mph = current_speedRPM * 60.0 * GETTF(TF_RADUMFANG_CM) / 100.0;  // current_speed in m pro h

																													 //flagShowSpeed= true;
	if (flagShowSpeed) {
		//if (millis() - lastTimeSpeedShown > 200) {
			lastTimeSpeedShown = millis();
			errorHandler.setInfoNoLog(F("!03,motor %i rpm: %f  m/h: %f deltaTicks: %lu deltaTime: %lu "), motorNo, current_speedRPM, current_speed_mph, encTickCounter, delta);
		//}
	}


}





/*+ ---------------------------------------------------------
Called at the loop rate to add "velocity" to the set point thus
effecting a motion.
Velocity is ramped up and down by "acceleration/deceleration"
-*/
void TClosedLoopControlThread::rampSetpoint(double  _sollSpeedRPM)
{
	if (useRamp) {

		if (setpointRPM < _sollSpeedRPM) {
			setpointRPM += rampAccRPM;

			if (_sollSpeedRPM > 0 && setpointRPM > 0 && setpointRPM < deadbandRPM) { // Wenn von speed 0 gestartet wird setpointRPM bei deadbandRPM gestartet um weniger träge zu sein. Speed von 5% =  _sollspeed: 57.000000
				setpointRPM = deadbandRPM;
				//debug->puts("setpointRPM = 20;\r\n");
			}
			if (setpointRPM > _sollSpeedRPM) {
				setpointRPM = _sollSpeedRPM;
			}

		}
		else if (setpointRPM > _sollSpeedRPM) {
			setpointRPM -= rampDecRPM;
			if (_sollSpeedRPM < 0 && setpointRPM < 0 && setpointRPM > (-deadbandRPM)) { // Wenn von speed 0 gestartet wird setpointRPM bei -20 anfangen
																	//debug->printf("setpointRPM %f, _sollspeed: %f\r\n", setpointRPM, _sollSpeed);
				setpointRPM = (-deadbandRPM);
				//debug->puts("setpointRPM = -20;\r\n");
			}
			if (setpointRPM < _sollSpeedRPM) {
				setpointRPM = _sollSpeedRPM;
			}

		}

	}//  if(useRamp) {
	else {
		setpointRPM = _sollSpeedRPM;
	}
	//debug << "_rampSetpoint( _sollSpeed)" << " _sollSpeed: " << _sollSpeed << "rampedSetpoint:  " << rampedSetpoint << " setpointRPM: " << setpointRPM << endl;
}


void TClosedLoopControlThread::hardStop()
{
	motorDriver.motor(motorNo, 0);
	sollSpeedRPM = 0;
	if (GetState() != STM_STOP_ROLLOUT && GetState() != STM_STOP) { // Verhindern, dass falls bereits in STM_STOP dieser nicht wieder in STM_STOP_ROLLOUT gesetzt wird.
																	// Dann kann es sein, dass bei Aufruf in loop nie in STOP kommmt, da immer STM_STOP_ROLLOUT gesetzt wird.
		SetState(STM_STOP_ROLLOUT);
		motorDriver.motor(motorNo, 0);
	}
}

void TClosedLoopControlThread::stop()
{
	sollSpeedRPM = 0;


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
	// Will only work if run is not called 

	if (speed < -127) speed = -127;
	else if (speed > 127) speed = 127;

	motorDriver.motor(motorNo, speed);

}


void TClosedLoopControlThread::showConfig()
{
	errorHandler.setInfoNoLog(F("!03,CLC Config MotorNo: %i\r\n"), motorNo);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
	errorHandler.setInfoNoLog(F("!03,KP: %f KI: %f  KD: %f\r\n"), myPID.GetKp(), myPID.GetKi(), myPID.GetKd());
	errorHandler.setInfoNoLog(F("!03,useRamp %d\r\n"), useRamp);
	errorHandler.setInfoNoLog(F("!03,rampAccRPM %f\r\n"), rampAccRPM);
	errorHandler.setInfoNoLog(F("!03,rampDecRPM %f\r\n"), rampDecRPM);
	errorHandler.setInfoNoLog(F("!03,deadbandRPM %f\r\n"), deadbandRPM);
	errorHandler.setInfoNoLog(F("!03,setOutputZeroAtRPm %f\r\n"), setOutputZeroAtRPm);
	errorHandler.setInfoNoLog(F("!03,stopThresholeAtRpm %f\r\n"), stopThresholeAtRpm);
	errorHandler.setInfoNoLog(F("!03,speedMinTest %f\r\n"), speedMinTest);
	errorHandler.setInfoNoLog(F("!03,speedMaxTest %f\r\n"), speedMaxTest);
}

/*
// FFarray is the array of expected motor velocities (units: sensorValue per second) that correspond with the duty cycle in increments of 20
static double FFarray[11] = { -118, -93, -70, -49, -26, 0, 26, 49, 70, 93, 118 };

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




void TClosedLoopControlThread::showEncoder()
{

	long encTickCounter, buff;
	unsigned long nowTime;
	double speedRPM, speed_mph;

	// Delta Encoder ermitteln
	buff = myEncoder->getTickCounter();
	encTickCounter = buff - lastTickCounterShowEnc;
	lastTickCounterShowEnc = buff;

	// Delta Time ermitteln 
	nowTime = millis();
	unsigned long delta = nowTime - lastTimeEncoderReadShowEnc;
	lastTimeEncoderReadShowEnc = nowTime;
	double deltaTimeSec = (double)delta / 1000.0;//time between most recent encoder ticks
	double ticksPerRevolution = GETTF(TF_ENCTICKSPERROTATION);

	if (delta != 0) {
		speedRPM = 60.0 * ((double)encTickCounter / ticksPerRevolution) / deltaTimeSec;
	}
	else {
		speedRPM = -999.99;
	}

	speed_mph = speedRPM * 60.0 * GETTF(TF_RADUMFANG_CM) / 100.0;  // current_speed in m pro h

	errorHandler.setInfoNoLog(F("!03,motor %i absEnc: %ld enc: %lu rpm: %f  m/h: %f deltaTicks: %lu deltaTime: %lu\r\n"), motorNo, myEncoder->getTickCounter(), myEncoder->getAbsTicksCounter(),speedRPM, speed_mph, encTickCounter, delta);

}

// Used for tuning the PID values
void TClosedLoopControlThread::motorStepSpeed()
{

	if ((millis() - lastrunTest) > 4000) {
		lastrunTest = millis();

		if (stateTest == 0) {
			setSpeed(speedMinTest);
			stateTest = 1;
		}
		else {
			setSpeed(speedMaxTest);
			stateTest = 0;
		}
	}
}



// Used for tuning the rampAccRPM and values
void TClosedLoopControlThread::testForwardStopBackward()
{

	unsigned long now = millis();


	switch (stateTest) {
	case 0:
		//forward();
		if ((now - lastrunTest) > 4000) {
			stateTest = 1;
			stop();
			errorHandler.setInfoNoLog(F("STOP\r\n"));
		}

		break;

	case 1:
		if (isStopped()) {
			lastrunTest = now;
			setSpeed(speedMinTest);
			stateTest = 2;
			errorHandler.setInfoNoLog(F("MIN\r\n"));
		}

		break;

	case 2:
		if ((now - lastrunTest) > 4000) {
			stateTest = 3;
			stop();
			errorHandler.setInfoNoLog(F("STOP\r\n"));
		}
		break;

	case 3:
		if (isStopped()) {
			lastrunTest = now;
			setSpeed(speedMaxTest);
			stateTest = 0;
			errorHandler.setInfoNoLog(F("MAX\r\n"));
		}
		break;

	default:
		break;
	}

}








