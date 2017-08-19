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

/*
How do I tune my system?

Start by setting the gains low.  You can set Ki and Kd to zero to start.

Increase Kp until the system starts to react quickly enough.  It will
overshoot if you set it too high.

Now increase Kd to compensate for overshoot.  Now the system should react
smoothly.

But you might notice that it never reaches the goal.  That is because
resistance in the system is holding it back and as you near the goal, the
proportional term gets smaller.  Now it is time to increase Ki.  Over time
the error will build and the I term allows the system to overcome resistance.

Now go back and adjust each of the terms to get the response you want.

========================================================================================================================

1. SET KP. Starting with KP=0, KI=0 and KD=0, increase KP until the output starts overshooting and ringing significantly.

2. SET KD. Increase KD until the overshoot is reduced to an acceptable level.

3. SET KI. Increase KI until the final error is equal to zero.

========================================================================================================================

Set all gains to zero.
Increase the P gain until the response to a disturbance is steady oscillation.
Increase the D gain until the the oscillations go away (i.e. it's critically damped).
Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
Set P and D to the last stable values.
Increase the I gain until it brings you to the setpoint with the number of oscillations desired (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot)

========================================================================================================================

The tuning of the three PID parameters KP, KI, and KD is an important issue. The following guidelines can be used for experimentally finding suitable values (adapted after [Williams 2006]):
1. Select a typical operating setting for the desired speed, turn off integral and derivative parts, then increase KP to maximum or until oscillation occurs.
2. If system oscillates, divide KP by 2.
3. Increase KD and observe behavior when increasing/decreasing the desired speed by about 5%. Choose a value of KD which gives a damped response.
4. Slowly increase KI until oscillation starts. Then divide KI by 2 or 3.
5. Check whether overall controller performance is satisfactorily under typical system conditions.

========================================================================================================================

Tuning PID

Once you have PID running in your robot, you will probably notice that it still doesn't
follow the line properly. It may even perform worse than it did with just proportional!
The reason behind this is you haven't tuned the PID routine yet.
PID requires the Kp, Ki and Kd factors to be set to match your robot's characteristics and
these values will vary considerably from robot to robot.  Unfortunately, there is no easy
way to tune PID. It requires manual trial and error until you get the desired behavior.
There are some basic guidelines that will help reduce the tuning effort.

Start with Kp, Ki and Kd equalling 0 and work with Kp first. Try setting Kp to a value of 1 and
observe the robot. The goal is to get the robot to follow the line even if it is very wobbly.
If the robot overshoots and loses the line, reduce the Kp value. If the robot cannot navigate a
turn or seems sluggish, increase the Kp value.
Once the robot is able to somewhat follow the line, assign a value of 1 to Kd
(skip Ki for the moment). Try increasing this value until you see less wobble.
Once the robot is fairly stable at following the line, assign a value of .5 to 1.0 to Ki.
If the Ki value is too high, the robot will jerk left and right quickly. If it is too low,
you won't see any perceivable difference.  Since Integral is cumulative, the Ki value has a
significant impact. You may end up adjusting it by .01 increments.
Once the robot is following the line with good accuracy, you can increase the speed and see
if it still is able to follow the line. Speed affects the PID controller and will require retuning
as the speed changes.

*/

#ifndef _PID_V2_h
#define _PID_V2_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#define LIBRARY_VERSION 2



//Constants used in some of the functions below
#define AUTOMATIC  1
#define MANUAL  0
#define DIRECT  0
#define REVERSE  1

// Position Pid
class PIDPOS
{

public:

	//commonly used functions **************************************************************************
	void  setup(double*, double*, double*,        // * setup.  links the PID to the Input, Output, and
		double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here

	void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

	bool Compute();                       // * performs the PID calculation.  it should be
										  //   called every time loop() cycles. ON/OFF and
										  //   calculation frequency can be set using SetMode
										  //   SetSampleTime respectively

	void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



										  //available but not commonly used functions ********************************************************
	void SetTunings(double, double,       // * While most users will set the tunings once in the
		double);              //   constructor, this function gives the user the option
							  //   of changing tunings during runtime for Adaptive control
	void SetKp(double Kp);
	void SetKi(double Ki);
	void SetKd(double Kd);

	void SetControllerDirection(int);     // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.

	bool Int_Improvement;
	bool Diff_Improvement;
	double Prev_AbsError;


	//Display functions ****************************************************************
	double GetKp();                       // These functions query the pid for interal values.
	double GetKi();                       //  they were created mainly for the pid front-end,
	double GetKd();                       // where it's important to know what is actually
	int GetMode();                        //  inside the PID.
	int GetDirection();                   //
	void Initialize();

private:

	double kp;                  // * (P)roportional Tuning Parameter
	double ki;                  // * (I)ntegral Tuning Parameter
	double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

	double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
	double *myOutput;             //   This creates a hard link between the variables and the
	double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
								  //   what these values are.  with pointers we'll just know.

	unsigned long lastTimeInMillis;
	double ITerm, lastInput, lastError;

	double outMin, outMax;
	bool inAuto;

	bool First_Time;
};

//Velocity Pid
class PIDVEL
{

public:
	bool flagShowPID;
	//commonly used functions **************************************************************************
	void  setup(double*, double*, double*,        // * setup.  links the PID to the Input, Output, and
		double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here

	void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

	bool Compute();                       // * performs the PID calculation.  it should be
										  //   called every time loop() cycles. ON/OFF and
										  //   calculation frequency can be set using SetMode
										  //   SetSampleTime respectively

	void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



										  //available but not commonly used functions ********************************************************
	void SetTunings(double, double,       // * While most users will set the tunings once in the
		double);             //   constructor, this function gives the user the option
							 //   of changing tunings during runtime for Adaptive control
	void SetKp(double Kp);
	void SetKi(double Ki);
	void SetKd(double Kd);

	void SetControllerDirection(int);   // * Sets the Direction, or "Action" of the controller. DIRECT
										//   means the output will increase when error is positive. REVERSE
										//   means the opposite.  it's very unlikely that this will be needed
										//   once it is set in the constructor.


										//Display functions ****************************************************************
	double GetKp();             // These functions query the pid for interal values.
	double GetKi();             //  they were created mainly for the pid front-end,
	double GetKd();             // where it's important to know what is actually
	int GetMode();              //  inside the PID.
	int GetDirection();           //
	void Initialize();

private:

	double kp;                  // * (P)roportional Tuning Parameter
	double ki;                  // * (I)ntegral Tuning Parameter
	double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

	double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
	double *myOutput;             //   This creates a hard link between the variables and the
	double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
								  //   what these values are.  with pointers we'll just know.

	unsigned long lastTimeInMillis;

	double outMin, outMax;
	bool inAuto;

	double input1;
	double input2;
	double error1;


	bool First_Time;
};




#endif


