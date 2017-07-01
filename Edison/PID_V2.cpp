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



/**********************************************************************************************
Arduino PID Library - Version 1.1.1
by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

This Library is licensed under a GPLv3 License

Changed for own needs kai Würtz
**********************************************************************************************/

#include "PID_V2.h"
#include <math.h>
#include "global.h"
#include "hardware.h"

/*Constructor (...)*********************************************************
The parameters specified here are those for for which we can't set up
reliable defaults, so we need to have the user set them.
***************************************************************************/
void PIDPOS::setup(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
{

	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	inAuto = false;

	SetOutputLimits(-127, 127);               //default output limit corresponds to sabertooth
	SetControllerDirection(ControllerDirection);
	SetTunings(Kp, Ki, Kd);


	Int_Improvement = false;
	Diff_Improvement = false;
	Prev_AbsError = 0;
	First_Time = true;
}



/* Compute() **********************************************************************
This, as they say, is where the magic happens.  this function should be called
every time "void loop()" executes. the function will decide for itself whether a new
pid Output needs to be computed.  returns true when the output is computed,
false when nothing has been done.
Duration on DUE for calculation 12ms

ACHTUNG abgeändert. Muss in bestimmten Zeitabständen aufgerufen werden
**********************************************************************************/
bool PIDPOS::Compute()
{
	if (!inAuto) return false;

	unsigned long now = millis();
	unsigned long delta = now - lastTimeInMillis;
	double Ta = (double)(delta) / 1000.0;  // Ta in seconds

										   /*Compute all the working error variables*/
	double input = *myInput;
	double error = *mySetpoint - input;


	if (First_Time) {  // First call initialize the variables for bumpless Transfer and return. If not using "return" in this if statement,then  Ta has to be defined and set to the used intertvall.
		First_Time = false;
		ITerm = *myOutput;
		lastInput = *myInput;
		lastError = error;
		lastTimeInMillis = now; // no Ta used. Get current time to calculate ad return.
		Prev_AbsError = 0;
		if (ITerm > outMax) ITerm = outMax;
		else if (ITerm < outMin) ITerm = outMin;
		return false;
	}


	double ErrAbs;
	if (Diff_Improvement) ErrAbs = fabs(error);

	// --- calculate proportional value ---
	double PTerm = kp * error;

	// --- PID Int Improvement ---
	if (Int_Improvement) {
		if (sign0plus(error) != sign0plus(ITerm))  ITerm = 0;  //sign0minus => 0 belongs to plus

	}

	// --- Calculate integrated value ---
	ITerm += (ki * error * Ta);

	if (ITerm > outMax) ITerm = outMax;
	else if (ITerm < outMin) ITerm = outMin;


	// --- calculate derivative value ---
	//double DTerm = kd / Ta *  (input - lastInput);
	double DTerm = kd / Ta *  (error - lastError);


	// --- PID Diff Improvement ---
	if (Diff_Improvement) {
		if (ErrAbs < Prev_AbsError)  DTerm = 0; // error becomes smaller, stop differentiation action
		Prev_AbsError = ErrAbs;
	}

	/*Compute PID Output*/
	//double output = PTerm + ITerm - DTerm;  // mind the minus sign!!!
	double output = PTerm + ITerm + DTerm;

	if (output > outMax) output = outMax;
	else if (output < outMin) output = outMin;
	*myOutput = output;

	//debug->printf("e: %f P: %f I: %f D: %f\r\n",error,PTerm,ITerm, DTerm);
	/*Remember some variables for next time*/
	lastInput = input;
	lastError = error;
	lastTimeInMillis = now;

	//debug << "Duration in mikroSek: " << micros() - now << endl;

	return true;


}


/* SetTunings(...)*************************************************************
This function allows the controller's dynamic performance to be adjusted.
it's called automatically from the constructor, but tunings can also
be adjusted on the fly during normal operation
******************************************************************************/
void PIDPOS::SetTunings(double Kp, double Ki, double Kd)
{
	if (Kp < 0 || Ki < 0 || Kd < 0) return;


	kp = Kp;
	ki = Ki;
	kd = Kd;

	if (controllerDirection == REVERSE) {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

void PIDPOS::SetKp(double Kp)
{
	if (Kp < 0) return;
	kp = Kp;
	if (controllerDirection == REVERSE) {
		kp = (0 - kp);
	}
}
void PIDPOS::SetKi(double Ki)
{
	if (Ki < 0) return;
	ki = Ki;
	if (controllerDirection == REVERSE) {
		ki = (0 - ki);
	}
}
void PIDPOS::SetKd(double Kd)
{
	if (Kd < 0) return;
	kd = Kd;
	if (controllerDirection == REVERSE) {
		kd = (0 - kd);
	}
}


/* SetOutputLimits(...)****************************************************
This function will be used far more often than SetInputLimits.  while
the input to the controller will generally be in the 0-1023 range (which is
the default already,)  the output will be a little different.  maybe they'll
be doing a time window and will need 0-8000 or something.  or maybe they'll
want to clamp it from 0-125.  who knows.  at any rate, that can all be done
here.
**************************************************************************/
void PIDPOS::SetOutputLimits(double Min, double Max)
{
	if (Min >= Max) return;
	outMin = Min;
	outMax = Max;

	if (inAuto) {
		if (*myOutput > outMax) *myOutput = outMax;
		else if (*myOutput < outMin) *myOutput = outMin;

		if (ITerm > outMax) ITerm = outMax;
		else if (ITerm < outMin) ITerm = outMin;
	}
}

/* SetMode(...)****************************************************************
Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
when the transition from manual to auto occurs, the controller is
automatically initialized
******************************************************************************/
void PIDPOS::SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto) {
		/*we just went from manual to auto*/
		Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
does all the things that need to happen to ensure a bumpless transfer
from manual to automatic mode.
******************************************************************************/
void PIDPOS::Initialize()
{
	First_Time = true;
}


/* SetControllerDirection(...)*************************************************
The PID will either be connected to a DIRECT acting process (+Output leads
to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
know which one, because otherwise we may increase the output when we should
be decreasing.  This is called from the constructor.
******************************************************************************/
void PIDPOS::SetControllerDirection(int Direction)
{
	if (inAuto && Direction != controllerDirection) {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controllerDirection = Direction;
}

/* Status Funcions*************************************************************
Just because you set the Kp=-1 doesn't mean it actually happened.  these
functions query the internal state of the PID.  they're here for display
purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
double PIDPOS::GetKp()
{
	return  kp;
}
double PIDPOS::GetKi()
{
	return  ki;
}
double PIDPOS::GetKd()
{
	return  kd;
}



int PIDPOS::GetMode()
{
	return  inAuto ? AUTOMATIC : MANUAL;
}
int PIDPOS::GetDirection()
{
	return controllerDirection;
}


/********************************************************************************/
/********************************************************************************/
/********************************************************************************/
// Velocity PID
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/

/*Constructor (...)*********************************************************
The parameters specified here are those for for which we can't set up
reliable defaults, so we need to have the user set them.
***************************************************************************/
void PIDVEL::setup(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
{

	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	inAuto = false;

	SetOutputLimits(-127, 127);        //default output limit corresponds to sabertooth
	SetControllerDirection(ControllerDirection);
	SetTunings(Kp, Ki, Kd);

	input1 = 0;
	input2 = 0;
	error1 = 0;
	First_Time = true;
}


bool PIDVEL::Compute()
{
	if (!inAuto) return false;

	unsigned long now = millis();
	unsigned long delta = now - lastTimeInMillis;
	double Ta = (double)(delta) / 1000.0;  // Ta in seconds

										   /*Compute all the working error variables*/
	double input = *myInput;
	double error = *mySetpoint - input;
	double output = *myOutput;


	if (First_Time) {  // First call
		First_Time = false;
		lastTimeInMillis = now;
		input1 = input;
		input2 = input;
		error1 = error;
		return false;
	}

	output = output
		+ kp * (error - error1)
		+ ki * Ta * error
		+ kd / Ta * (input - 2 * input1 + input2);

	// restrict output to min/max
	if (output > outMax) output = outMax;
	if (output < outMin) output = outMin;


	//debug->printf("*mySetpoint: %f input: %f error: %f output: %f Ta: %f PTerm: %f ITerm: %f DTerm: %f\r\n",*mySetpoint, input, error, output,Ta,(kp * (error - error1)) , (ki * Ta * error)),(kd / Ta * (input - 2 * input1 + input2));

	// save variable for next time
	input2 = input1;
	input1 = input;
	error1 = error;

	*myOutput = output;

	lastTimeInMillis = now;
	//  now = micros() - now;
	//  debug << "Duration in microSec: " <<  now << endl;

	return true;

}

/* SetTunings(...)*************************************************************
This function allows the controller's dynamic performance to be adjusted.
it's called automatically from the constructor, but tunings can also
be adjusted on the fly during normal operation
******************************************************************************/
void PIDVEL::SetTunings(double Kp, double Ki, double Kd)
{
	if (Kp < 0 || Ki < 0 || Kd < 0) return;


	kp = Kp;
	ki = Ki;
	kd = Kd;

	if (controllerDirection == REVERSE) {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

void PIDVEL::SetKp(double Kp)
{
	if (Kp < 0) return;
	kp = Kp;
	if (controllerDirection == REVERSE) {
		kp = (0 - kp);
	}
}
void PIDVEL::SetKi(double Ki)
{
	if (Ki < 0) return;
	ki = Ki;
	if (controllerDirection == REVERSE) {
		ki = (0 - ki);
	}
}
void PIDVEL::SetKd(double Kd)
{
	if (Kd < 0) return;
	kd = Kd;
	if (controllerDirection == REVERSE) {
		kd = (0 - kd);
	}
}



/* SetOutputLimits(...)****************************************************
This function will be used far more often than SetInputLimits.  while
the input to the controller will generally be in the 0-1023 range (which is
the default already,)  the output will be a little different.  maybe they'll
be doing a time window and will need 0-8000 or something.  or maybe they'll
want to clamp it from 0-125.  who knows.  at any rate, that can all be done
here.
**************************************************************************/
void PIDVEL::SetOutputLimits(double Min, double Max)
{
	if (Min >= Max) return;
	outMin = Min;
	outMax = Max;

	if (inAuto) {
		if (*myOutput > outMax) *myOutput = outMax;
		else if (*myOutput < outMin) *myOutput = outMin;
	}
}

/* SetMode(...)****************************************************************
Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
when the transition from manual to auto occurs, the controller is
automatically initialized
******************************************************************************/
void PIDVEL::SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto) {
		/*we just went from manual to auto*/
		Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
does all the things that need to happen to ensure a bumpless transfer
from manual to automatic mode.
******************************************************************************/
void PIDVEL::Initialize()
{
	First_Time = true;
}

/* SetControllerDirection(...)*************************************************
The PID will either be connected to a DIRECT acting process (+Output leads
to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
know which one, because otherwise we may increase the output when we should
be decreasing.  This is called from the constructor.
******************************************************************************/
void PIDVEL::SetControllerDirection(int Direction)
{
	if (inAuto && Direction != controllerDirection) {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controllerDirection = Direction;
}

/* Status Funcions*************************************************************
Just because you set the Kp=-1 doesn't mean it actually happened.  these
functions query the internal state of the PID.  they're here for display
purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
double PIDVEL::GetKp()
{
	return  kp;
}
double PIDVEL::GetKi()
{
	return  ki;
}
double PIDVEL::GetKd()
{
	return  kd;
}


int PIDVEL::GetMode()
{
	return  inAuto ? AUTOMATIC : MANUAL;
}
int PIDVEL::GetDirection()
{
	return controllerDirection;
}




