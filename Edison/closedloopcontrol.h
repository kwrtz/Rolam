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


#ifndef _CLOSEDLOOPCONTROL_h
#define _CLOSEDLOOPCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



#include "Thread.h"
#include "PID_v2.h"
#include "helpers.h"
#include "Sabertooth.h"
#include "hardware.h"
#include "errorhandler.h"


//static const unsigned long ENCTIMEOUT = 200000ul;
//#define   MAXSPEED 1140l                // MAXSPEED maximale Geschwindigkeit (ticks pro sek). Motor erreicht eine Geschwindigkeit von 1470 Ticks/sek bei motorpower von 127. Wird in Funktion setspeed verwendet. 1140 bedeuten 100%.
//static const double  TICKSPERCM = 26.9926f;           // Anzahl encoderticks per cm
//#define  TICKSPERM  2699.26f




enum EMotorState {
	STM_OFF = -1, // optional, -1 is the initial state of the fsm
	STM_RUN,
	STM_STOP_REQUEST,
	STM_STOP_ROLLOUT,
	STM_STOP
};

class TClosedLoopControlThread : public Thread, public FSM<EMotorState>
{
private:

	virtual void UpdateState(EMotorState t);
	virtual void BeginState(EMotorState t);
	virtual void EndState(EMotorState t);



	unsigned long lastTimeEncoderRead;
	unsigned long lastTimeSpeedShown;
	unsigned long timeLastCheckMotorStall;


	long lastTickCounter;
	unsigned long lastAbsTickCounter;
	void readEncoder(); // reads encodere and calcualtes current_speed

						// PID input and und parameter

	double setpointRPM, current_speedRPM, Output; // Pid setpoint, input und oputput
	double feedforwardRPM;


	// ramping the setpoint with acceleration values
	void rampSetpoint(double  _sollSpeed);  //Setpoint wird auf _sollSpeed gerampt. Dazu wird bei jedem aufruf acceleration abgezogen oder dazugezählt.


	double  sollSpeedRPM; // Soll speed wird von setSpeed gesetzt. EncoderTicks/Sek

	// Test functions
	unsigned long lastTickCounterShowEnc;
	unsigned long lastTimeEncoderReadShowEnc;
	

public:

	uint8_t motorNo;  // Motornumber welcher motor soll von sabertooh angesprochen werden

	PIDVEL myPID;

	CRotaryEncoder *myEncoder;



	bool useRamp;

	float rampAccRPM; // Constant for acceleration
	float rampDecRPM; // Constant for deceleration
	float deadbandRPM; // In diesem Bereich wird setpointRPM auf deadbandRPM gesetzt um die Agilität beim Starten der Rampe zu erhöhen.
	float setOutputZeroAtRPm; // If sollSpeedRPM should be 0, this is the threshold  where output and feedforward is set to zero to improve agility. Positive value!
	float stopThresholeAtRpm; // If the motor is running at this threshold, it is asssumed, that th motor stands while waiting in state STM_STOP_ROLLOUT
	float current_speed_mph;

	void setup(uint8_t motorNumber, CRotaryEncoder *enc);

	void setSpeed(long  speed);  // -100% bis 100% Motorgeschwindigkeit festlegen und sofort losfahren

	void stop(); // motor stoppen
	void hardStop();
	bool isStopped();

	void enableDefaultRamping();
	void enablePerTrackRamping();
	void enableFastStopRamping();

	//void disableRamping();

	virtual void run();  // Called at the loop rate 20ms to run the state machine and therfore the motor

						 // Sabertooth Befehle
	void setRamp();
	void setBaudRate();

	bool flagShowSpeed; // show speed on debug interface

	void controlDirect(int speed); // set direct speed through sabertooth -127 to 127

	void showConfig();

	// Test functions
	//----------------
	unsigned long lastrunTest; // used for PID tune

	int stateTest;
	int speedMinTest, speedMaxTest; //speed for PID tune

	bool flagShowEncoder; // show encoder on debug interface
	void showEncoder();

	bool flagControldirect;

	bool flagMotorStepSpeed; // accelerate and decelerate the motor form 80% to 60 %
	void motorStepSpeed();

	bool flagMotorFSB; // drive forward to 80% stop drive backward to -80% stop
	void testForwardStopBackward();

};



#endif

