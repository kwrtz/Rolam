/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz

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

#include "ui.h"

#include "hardware.h"
#include "cmd.h"
#include "perimeter.h"
#include "batterySensor.h"
#include "motor.h"
#include "closedloopcontrol.h"
#include "mowclosedloopcontrol.h"
#include "errorhandler.h"
#include "mowmotorSensor.h"
#include "rangeSensor.h"
#include "bumperSensor.h"
#include "Blackboard.h"
#include "behaviour.h"
#include "bPerimeterTracking.h"
#include "chargeSystem.h"
#include "bPerOutside.h"
#include "tables.h"
#include "bt.h"
#include "adcman.h"
#include "rtc.h"
#include "EEPROM.h"

extern void executeLoop();

extern unsigned long lastTimeShowError;
extern TRotateWorkx rotateWorkx;

// mow motor closed loop control - no closed loop used
extern TMowClosedLoopControlThread clcM;
// drive motor left closed loop control
extern TClosedLoopControlThread clcL;
// drive motor rigth closed loop control
extern TClosedLoopControlThread clcR;
//drive motor left position control
extern TPositionControl pcL;
//drive motor right position control
extern TPositionControl pcR;
// Motorensteuerung Interface. KEIN THREAD. Wird verwendet um clcX und pcX zu steuern.
extern TMotorInterface motor;
// Daten von Perimetersensoren vom 446RE
extern TPerimeterThread perimeterSensoren;
// Messung der Batteriespannung
extern TbatterieSensor batterieSensor;
// Messung des MÃ¤hmotorstroms
extern TMowMotorSensor mowMotorSensor;
// SRF08 Range Sensor Messung der Entfernung
extern TrangeSensor rangeSensor;
// Bumper Sensor
extern TbumperSensor bumperSensor;
// Charge System
extern TchargeSystem chargeSystem;

extern Blackboard myBlackboard;
extern TBehaviour myBehaviour;
extern TtrackPerimeter trackPerimeter;

extern TErrorHandler errorHandler;

extern Trtc rtc;
extern TEEPROM eeprom;

extern void FreeMem(void);



bool checkManualMode() {
	if (GETTB(TB_CONTROL_MANUAL) != true) {
		errorHandler.setInfoNoLog(F("!03,NEED TO BE IN MANUAL MODE!\r\n"));
		return false;
	}

	return true;
}
/********************************************************************************
********************************************************************************
**  functions for user commands
**  each function represents one user command
********************************************************************************
*********************************************************************************/

void cmd_help(int arg_cnt, char **args)
{
	unsigned long wait;

	errorHandler.setInfoNoLog(F("Edison interface emulator\r\n"));
	errorHandler.setInfoNoLog(F("=========================\r\n"));
	errorHandler.setInfoNoLog(F("Available debug commands: (lines end with CRLF or '\\r')\r\n"));
	errorHandler.setInfoNoLog(F("H will print this help message again\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("hello print hello message\r\n"));
	errorHandler.setInfoNoLog(F("args,1,2,3 show args 1 2 3\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();
	
	errorHandler.setInfoNoLog(F("\r\n=== MODE SELECTION ===\r\n"));
	errorHandler.setInfoNoLog(F("A //automatic control over actuators\r\n"));
	errorHandler.setInfoNoLog(F("M //manuel control over actuators\r\n"));
	errorHandler.setInfoNoLog(F("area,12 //drive 12m at perimeter and begin mowing\r\n"));
	errorHandler.setInfoNoLog(F("gohome //drive to docking station. Call again to deactivate\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("\r\n=== ERROR HANDLING ===\r\n"));
	errorHandler.setInfoNoLog(F("error //shows errormessage\r\n"));
	errorHandler.setInfoNoLog(F("reset //reset error and motor faults\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();


	errorHandler.setInfoNoLog(F("\r\n=== BLUETOOTH ===\r\n"));
	errorHandler.setInfoNoLog(F("bt.show //try to detect BT module\r\n"));
	errorHandler.setInfoNoLog(F("bt.set //configure BT module\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("\r\n=== I2C/RTC//EEPROM SERVICE ===\r\n"));
	errorHandler.setInfoNoLog(F("i2c.scan //i2c scanner\r\n"));
	errorHandler.setInfoNoLog(F("rtc.show //shows rtc values every rtc read (10sec)\r\n"));
	errorHandler.setInfoNoLog(F("rtc.find //tries to find RTC and show result\r\n"));
	errorHandler.setInfoNoLog(F("rtc.set,8,17,3,25,01,2017 //set rtc time=8:17 dayOfWeek=3 date=25.01.2017\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("eep.u8t,10 //shows uint8_t at address 10\r\n"));
	errorHandler.setInfoNoLog(F("eep.s32t,10 //shows int32_t at address 10\r\n"));
	errorHandler.setInfoNoLog(F("eep.f,10 //shows float at address 10\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("eep.set.u8t,10,7 //write value uint8_t=7 to address=10 \r\n"));
	errorHandler.setInfoNoLog(F("eep.set.s32t,10,1234 //write value int32_t=1234 to address=10 \r\n"));
	errorHandler.setInfoNoLog(F("eep.set.f,10,7.3 //write value float=7.3 to address=10 \r\n"));


	wait = millis();
	while (millis() - wait < 100) executeLoop();


	errorHandler.setInfoNoLog(F("\r\n=== DRIVE MOTOR CLOSED LOOP CONTROL SERVICE ===\r\n"));
	errorHandler.setInfoNoLog(F("clc.config //shows clcL/R config\r\n"));
	errorHandler.setInfoNoLog(F("clc.enc    //show encoder values \r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("clc.pidl //show pid calcualtion left\r\n"));
	errorHandler.setInfoNoLog(F("clc.pidr //show pid calcualtion right\r\n"));
	errorHandler.setInfoNoLog(F("clc.speedl //show speed left\r\n"));
	errorHandler.setInfoNoLog(F("clc.speedr //show speed right\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("clc.v,30  //drives both motors in closed loop with speed of 30%\r\n"));
	errorHandler.setInfoNoLog(F("          //value: -100%% to 100%%\r\n"));
	errorHandler.setInfoNoLog(F("clc.s     //stop drive motors\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("clc.p,123.34  //sets drive motors proportional term\r\n"));
	errorHandler.setInfoNoLog(F("clc.i,123.34 //sets drive motors integral term\r\n"));
	errorHandler.setInfoNoLog(F("clc.d,123.34 //sets drive motors derivative term\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("clc.ramp,1.3,1.5 //sets ramp acceleration=1.3, deceleration=1.5 in RPM\r\n"));
	errorHandler.setInfoNoLog(F("clc.ag,2.0,3.0,1.0 //sets agility deadbandRPM=2.0, setOutputZeroAtRPm=3.0 stopThresholdAtRpm=1.0 in RPM\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("clc.mt,1,30 //direct motor test. run motor=1 with speed=30\r\n"));
	errorHandler.setInfoNoLog(F("        //motor: 1=L, 2=R  speed= -127 to 127\r\n"));
	errorHandler.setInfoNoLog(F("        //deactivates closed loop control\r\n"));
	errorHandler.setInfoNoLog(F("        //end test with: clc.mt,0,0\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("clc.msp,60,80,0 //steps both drive motors from 60%% to 80%%  0/1=without/with ramp\r\n"));
	errorHandler.setInfoNoLog(F("            //end test with: clc.msp,0,0,0\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("clc.msps,-60,80,1 //steps both drive motor from -60%% to 80%%  0/1=without/with ramp\r\n"));
	errorHandler.setInfoNoLog(F("              //stops first before run to next speed\r\n"));
	errorHandler.setInfoNoLog(F("              //end test with: clc.msps,0,0,0\r\n"));


	wait = millis();
	while (millis() - wait < 100) executeLoop();
	
	errorHandler.setInfoNoLog(F("\r\n=== MOW MOTOR CLOSED LOOP CONTROL SERVICE ===\r\n"));
	errorHandler.setInfoNoLog(F("clcm.config //shows clcM config\r\n"));
	errorHandler.setInfoNoLog(F("clcm.speed //show speed 0-127 \r\n"));
	errorHandler.setInfoNoLog(F("clcm.accel,2000 //set ramp factor 2000. The higher the slower the acc.\r\n"));
	errorHandler.setInfoNoLog(F("clcm.limit,117 //set speedLimit to 117  Values: 0-127\r\n"));
	
    errorHandler.setInfoNoLog(F("z //mow motor start\r\n"));
	errorHandler.setInfoNoLog(F("t //mow motor stop\r\n"));



	errorHandler.setInfoNoLog(F("\r\n=== POSITION CONTROL SERVICE ===\r\n"));
	errorHandler.setInfoNoLog(F("pc.config //shows pcL/R config\r\n"));
	errorHandler.setInfoNoLog(F("pc.L //show result after pos reached\r\n"));
	errorHandler.setInfoNoLog(F("pc.R //show result after pos reached\r\n"));
	errorHandler.setInfoNoLog(F("pc.tuneup,2.1,5.2,3.1 //posKp,stopCmBeforeTarget,addCmToTargetPosition\r\n"));
	errorHandler.setInfoNoLog(F("pc.a,60,30 //rotate wheel 60 degrees with speed 30\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("pc.cm,60,30 //drives 60 cm with speed 30\r\n"));
	errorHandler.setInfoNoLog(F("            //negative drives backward\r\n"));
	errorHandler.setInfoNoLog(F("pc.stop //stop Positioning\r\n"));





	errorHandler.setInfoNoLog(F("set.ghpid,0.1 0.2 0.3 //set line follower pid kp ki kd\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("set.relay,1/0 //turn relay on/off\r\n"));
	errorHandler.setInfoNoLog(F("set.proc,1/0  //turn output for processing on/off\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("set.mowmode,0/1  //0=Workx 1=308\r\n"));
	errorHandler.setInfoNoLog(F("set.spiral,0/1  //0=Off 1=On\r\n"));
	errorHandler.setInfoNoLog(F("set.f,1,10 //set f table value idx=1 value=10\r\n"));
	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("set.i,1,10 //set i table value idx=1 value=10\r\n"));
	errorHandler.setInfoNoLog(F("set.b,1,0/1 //set b table value idx=1 value = 0 or 1\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();






	errorHandler.setInfoNoLog(F("\n\r\nMANUAL MODE COMMANDS\r\n"));



	errorHandler.setInfoNoLog(F("turnto,60,30  //turn 60 degrees right with speed 30\r\n"));
	errorHandler.setInfoNoLog(F("              //negative turns left\r\n"));


	wait = millis();
	while (millis() - wait < 100) executeLoop();




	errorHandler.setInfoNoLog(F("show.bat //shows battery voltage\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("show.per //shows perimeter sensor values\r\n"));
	errorHandler.setInfoNoLog(F("show.permax //shows maximum perimeter value\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();




	errorHandler.setInfoNoLog(F("show.mowsens //show mowsensor\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();


	errorHandler.setInfoNoLog(F("show.range //show rangesensor values\r\n"));
	errorHandler.setInfoNoLog(F("show.bumper //show bumper sensor if active\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("show.charge //shows charge sensors\r\n"));
	errorHandler.setInfoNoLog(F("show.mem //shows free memory\r\n"));
	errorHandler.setInfoNoLog(F("show.distance //shows distance while drving to areaX\r\n"));

	wait = millis();
	while (millis() - wait < 100) executeLoop();
	errorHandler.setInfoNoLog(F("show.workx //shows workx rotate values\r\n"));
	errorHandler.setInfoNoLog(F("show.tables //shows config tables\r\n"));




	wait = millis();
	while (millis() - wait < 100) executeLoop();

	errorHandler.setInfoNoLog(F("h //hide showing\r\n"));


	errorHandler.setInfoNoLog(F("show.adc //shows adc config\r\n"));
	errorHandler.setInfoNoLog(F("show.adcv //shows adc values\r\n"));











}



// Closed loop control comands

void cmd_clc_setKP(int arg_cnt, char **args)
{
	float val = cmdStr2Float(args[1]);
	clcL.myPID.SetKp(val);
	clcR.myPID.SetKp(val);
}

void cmd_clc_setKI(int arg_cnt, char **args)
{
	float val = cmdStr2Float(args[1]);
	clcL.myPID.SetKi(val);
	clcR.myPID.SetKi(val);
}

void cmd_clc_setKD(int arg_cnt, char **args)
{
	float val = cmdStr2Float(args[1]);
	clcL.myPID.SetKd(val);
	clcR.myPID.SetKd(val);
}

void cmd_clc_show_config(int arg_cnt, char **args)
{
	clcL.showConfig();
	clcR.showConfig();
}

void cmd_clc_showSpeedL(int arg_cnt, char **args)
{
	clcL.flagShowSpeed = !clcL.flagShowSpeed;
}

void cmd_clc_showSpeedR(int arg_cnt, char **args)
{
	clcR.flagShowSpeed = !clcR.flagShowSpeed;
}

void cmd_clc_showPIDL(int arg_cnt, char **args)
{
	clcL.myPID.flagShowPID = !clcL.myPID.flagShowPID;
}


void cmd_clc_showPIDR(int arg_cnt, char **args)
{
	clcR.myPID.flagShowPID = !clcR.myPID.flagShowPID;
}

void cmd_clc_setSpeed(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		int val = cmdStr2Num(args[1], 10);
		clcL.setSpeed(val);
		clcR.setSpeed(val);
	}
}

void cmd_clc_setRamp(int arg_cnt, char **args)
{
	float val = cmdStr2Float(args[1]);
	float val1 = cmdStr2Float(args[2]);
	clcL.rampAccRPM = val;
	clcL.rampDecRPM = val1;	
	clcR.rampAccRPM = val;
	clcR.rampDecRPM = val1;

}


void cmd_clc_setAgility(int arg_cnt, char **args)
{
	float val = cmdStr2Float(args[1]);
	float val1 = cmdStr2Float(args[2]);
	float val2 = cmdStr2Float(args[3]);
	clcL.deadbandRPM = val;
	clcL.setOutputZeroAtRPm = val1;
	clcL.stopThresholeAtRpm = val2;
	clcR.deadbandRPM = val;
	clcR.setOutputZeroAtRPm = val1;
	clcR.stopThresholeAtRpm = val2;

}



void cmd_clc_driveStop(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		//motor.stop();
		clcL.stop();
		clcR.stop();
	}
}

void cmd_clc_showEncoder(int arg_cnt, char **args)
{
	clcL.flagShowEncoder = !clcL.flagShowEncoder;
	clcR.flagShowEncoder = !clcR.flagShowEncoder;
}

void cmd_clc_motorTest(int arg_cnt, char **args)
{
	int mot = cmdStr2Num(args[1],10);
	int val = cmdStr2Num(args[2],10);
	
	if (checkManualMode()) {

		if (val == 0) {
			clcL.flagControldirect = false;
			clcR.flagControldirect = false;
			clcL.controlDirect(val);
			clcR.controlDirect(val);
			clcL.stop();
			clcR.stop();
			return;
		}
		if (mot == 1) {
			clcL.flagControldirect = true;
			motorDriver.resetFault();
			clcL.controlDirect(val);
		}
		if (mot == 2) {
			clcR.flagControldirect = true;
			motorDriver.resetFault();
			clcR.controlDirect(val);
		}

	}
}




void cmd_clc_motorStepSpeed(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {

		clcL.speedMinTest = cmdStr2Num(args[1], 10);
		clcL.speedMaxTest = cmdStr2Num(args[2], 10);
		bool ramp = cmdStr2Num(args[3], 10);

		clcR.speedMinTest = clcL.speedMinTest;
		clcR.speedMaxTest = clcL.speedMaxTest;
		clcL.stateTest = 0;
		clcR.stateTest = 0;
		clcL.lastrunTest = millis() - 10000ul;
		clcR.lastrunTest = millis() - 10000ul;

		// Turn off test
		if (clcL.speedMinTest == 0 && clcL.speedMaxTest == 0) {
			clcL.flagMotorStepSpeed = false;
			clcR.flagMotorStepSpeed = false;
			clcL.useRamp = true;
			clcR.useRamp = true;
			clcL.stop();
			clcR.stop();
		}
		// Turn on test
		else {
			if (ramp == 0) {
				clcL.useRamp = false;
				clcR.useRamp = false;
			}
			else {
				clcL.useRamp = true;
				clcR.useRamp = true;
			}
			clcL.flagMotorStepSpeed = true;
			clcR.flagMotorStepSpeed = true;
		}
	}
}

void cmd_clc_motorFSB(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {

		clcL.speedMinTest = cmdStr2Num(args[1], 10);
		clcL.speedMaxTest = cmdStr2Num(args[2], 10);
		bool ramp = cmdStr2Num(args[3], 10);

		clcR.speedMinTest = clcL.speedMinTest;
		clcR.speedMaxTest = clcL.speedMaxTest;
		clcL.stateTest = 0;
		clcR.stateTest = 0;
		clcL.lastrunTest = millis() - 10000ul;
		clcR.lastrunTest = millis() - 10000ul;

		// Turn off test
		if (clcL.speedMinTest == 0 && clcL.speedMaxTest == 0) {
			clcL.flagMotorFSB = false;
			clcR.flagMotorFSB = false;
			clcL.useRamp = true;
			clcR.useRamp = true;
			clcL.stop();
			clcR.stop();
		}
		// Turn on test
		else {
			if (ramp == 0) {
				clcL.useRamp = false;
				clcR.useRamp = false;
			}
			else {
				clcL.useRamp = true;
				clcR.useRamp = true;
			}
			clcL.flagMotorFSB = true;
			clcR.flagMotorFSB = true;
		}
	}
}



void cmd_clcM_motorTest(int arg_cnt, char **args)
{
	int mot = cmdStr2Num(args[1], 10);
	int val = cmdStr2Num(args[2], 10);
	if (checkManualMode()) {

		if (val == 0) {
			clcM.enabled = true;
			mowMotorDriver.motor(1, val);
			clcM.stop();
			return;
		}

		if (mot == 1) {
			clcM.enabled = false;
			mowMotorDriver.resetFault();
			mowMotorDriver.motor(1, val);
		}

	}
}



void cmd_clcm_show_config(int arg_cnt, char **args)
{
	clcM.showConfig();
}

void cmd_clcm_showSpeed(int arg_cnt, char **args)
{
	clcM.flagShowSpeed = !clcM.flagShowSpeed;
}

void cmd_clcm_setRamp(int arg_cnt, char **args)
{
	int val = cmdStr2Num(args[1],10);
	clcM.motorMowAccel = val;
}

void cmd_clcm_setSpeedLimit(int arg_cnt, char **args)
{
	float val = cmdStr2Float(args[1]);
	clcM.speedLimit = val;
}

void cmd_pc_setTuneup(int arg_cnt, char **args)
{
	pcL.posKp = cmdStr2Float(args[1]);
	pcL.stopCmBeforeTarget = cmdStr2Float(args[2]);
	pcL.addCmToTargetPosition = cmdStr2Float(args[3]);

	pcR.posKp = pcL.posKp;
	pcR.stopCmBeforeTarget = pcL.stopCmBeforeTarget;
	pcR.addCmToTargetPosition = pcL.addCmToTargetPosition;
}

void cmd_pos_show_config(int arg_cnt, char **args)
{
	pcL.showConfig();
	pcR.showConfig();
}

void cmd_pos_show_resultsL(int arg_cnt, char **args)
{
	pcL.flagShowResults = !pcL.flagShowResults;
}

void cmd_pos_show_resultsR(int arg_cnt, char **args)
{
	pcR.flagShowResults = !pcR.flagShowResults;
}




void cmd_goHome(int arg_cnt, char **args)
{
	myBlackboard.flagGoHome = true;
}

void cmd_driveAngle(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		float winkel = cmdStr2Float(args[1]);
		float speed = cmdStr2Float(args[2]);
		motor.rotateAngle(winkel, speed);
	}
}

void cmd_driveCM(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		float cm = cmdStr2Float(args[1]);
		float speed = cmdStr2Float(args[2]);
		motor.rotateCM(cm, speed);
	}
}

void cmd_turnTo(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		float winkel = cmdStr2Float(args[1]);
		float speed = cmdStr2Float(args[2]);
		motor.turnTo(winkel, speed);
	}
}

void cmd_stopPositioning(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		motor.stop();
	}
}


void cmd_startMowMot(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		motor.mowMotStart();
		motor.M->uiMotorDisabled = false;
		motor.M->motorDisabled = false;
	}
	else {
		motor.M->uiMotorDisabled = false;
	}
}

void cmd_stopMowMot(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		motor.mowMotStop();
	}
	else {
		motor.M->uiMotorDisabled = true;
	}
}

void cmd_cntrManuel(int arg_cnt, char **args)
{
	SETTB(TB_CONTROL_MANUAL);
	motor.stopAllMotors();
	myBehaviour.reset();

}

void cmd_cntrAuto(int arg_cnt, char **args)
{
	if (myBlackboard.flagEnableMowing == false) {
		CLRTB(TB_CONTROL_MANUAL);
		myBehaviour.reset();
		myBlackboard.setBehaviour(BH_MOW);
	}

}

void cmd_cntrGotoAreaX(int arg_cnt, char **args)
{
	CLRTB(TB_CONTROL_MANUAL);
	myBehaviour.reset();
	long distance = cmdStr2Num(args[1], 10);
	myBlackboard.areaTargetDistanceInMeter = distance;
	myBlackboard.setBehaviour(BH_GOTOAREA);
	errorHandler.setInfoNoLog(F("Drive to area: %l\r\n"), distance);

}



void cmd_showBattery(int arg_cnt, char **args)
{
	errorHandler.setInfoNoLog(F("Battery Voltage: %f sensorValue: %f\r\n"), batterieSensor.voltage, batterieSensor.sensorValue);
	errorHandler.setInfoNoLog(F("aiBATVOLT.read_int32() %d\r\n"), aiBATVOLT.read_int32());

	//errorHandler.setInfoNoLog(F( "aiBATVOLT.getVoltage() %f\r\n"), aiBATVOLT.getVoltage());
}

void cmd_showMowSensor(int arg_cnt, char **args)
{
	mowMotorSensor.showValuesOnConsole = true;
}


void cmd_showPerimeter(int arg_cnt, char **args)
{
	perimeterSensoren.showValuesOnConsole = true;
}

void cmd_showPerimeterMax(int arg_cnt, char **args)
{
	errorHandler.setInfoNoLog(F("magMax:   %ld\r\n"), perimeterSensoren.magMax);
}


void cmd_showRTC(int arg_cnt, char **args)
{
	errorHandler.setInfoNoLog(F("millis():   %lu\r\n"), millis());

	errorHandler.setInfoNoLog(F("Current RTC:\r\n"));
	rtc.showImmediately();

	rtc.readDS1307();
	errorHandler.setInfoNoLog(F("Read from RTC:\r\n"));
	rtc.showImmediately();
	rtc.flagShowRTCRead = !rtc.flagShowRTCRead;
	//errorHandler.setInfoNoLog(F( "micros():   %lu\r\n",micros());
	//errorHandler.setInfoNoLog(F( "micros64(): %llu\r\n",micros64());
	//errorHandler.setInfoNoLog(F( "millis64(): %llu\r\n",millis64());
	//errorHandler.setInfoNoLog(F( "Sekunden(): %llu\r\n",micros64()/1000000ULL);
	//errorHandler.setInfoNoLog(F( "Minuten():  %llu\r\n",micros64()/1000000ULL/60ULL);
}

void cmd_showRTCfind(int arg_cnt, char **args)
{
	if (checkManualMode()) {
		rtc.findDS1307();
	}
}


void cmd_setRTC(int arg_cnt, char **args)
{
	if (checkManualMode()) {
		rtc.hour = cmdStr2Num(args[1], 10);;
		rtc.minute = cmdStr2Num(args[2], 10);;
		rtc.dayOfWeek = cmdStr2Num(args[3], 10);;
		rtc.day = cmdStr2Num(args[4], 10);;
		rtc.month = cmdStr2Num(args[5], 10);;
		rtc.year = cmdStr2Num(args[6], 10);;
		rtc.showImmediately();
		errorHandler.setInfoNoLog(F("saving...\r\n"));
		rtc.setDS1307();
		errorHandler.setInfoNoLog(F("saved\r\n"));
		delay(500);
		errorHandler.setInfoNoLog(F("reading from rtc...\r\n"));
		rtc.readDS1307();
		rtc.showImmediately();
		errorHandler.setInfoNoLog(F("read\r\n"));

	}
}


void cmd_setEEPROMbyte(int arg_cnt, char **args)
{
	if (checkManualMode()) {
		int16_t address = cmdStr2Num(args[1], 10);
		int8_t data = cmdStr2Num(args[2], 10);

		eeprom.write(address, data);
		errorHandler.setInfoNoLog(F("saved\r\n"));
	}

}

void cmd_setEEPROM32t(int arg_cnt, char **args)
{
	if (checkManualMode()) {
		int16_t address = cmdStr2Num(args[1], 10);
		int32_t data = cmdStr2Num(args[2], 10);

		eeprom.write32t(address, data);
		errorHandler.setInfoNoLog(F("saved\r\n"));
	}

}

void cmd_setEEPROMfloat(int arg_cnt, char **args)
{
	if (checkManualMode()) {
		int16_t address = cmdStr2Num(args[1], 10);
		float data = cmdStr2Float(args[2]);

		eeprom.writeFloat(address, data);
		errorHandler.setInfoNoLog(F("saved\r\n"));
	}

}

void cmd_showEEPROMbyte(int arg_cnt, char **args)
{
	uint8_t data;

	if (checkManualMode()) {
		int16_t address = cmdStr2Num(args[1], 10);

		int i = eeprom.read(address, data);
		if (i > 0) {
			errorHandler.setInfoNoLog(F("Value: %d\r\n"), data);
		}
		else {
			errorHandler.setInfoNoLog(F("reading failed\r\n"));
		}

	}
}


void cmd_showEEPROM32t(int arg_cnt, char **args)
{
	int32_t data;

	if (checkManualMode()) {
		int16_t address = cmdStr2Num(args[1], 10);

		int i = eeprom.read32t(address, data);
		if (i > 0) {
			errorHandler.setInfoNoLog(F("Value: %d\r\n"), data);
		}
		else {
			errorHandler.setInfoNoLog(F("reading failed\r\n"));
		}

	}
}

void cmd_showEEPROMfloat(int arg_cnt, char **args)
{
	float data;

	if (checkManualMode()) {
		int16_t address = cmdStr2Num(args[1], 10);

		int i = eeprom.readFloat(address, data);
		if (i > 0) {
			errorHandler.setInfoNoLog(F("Value: %f\r\n"), data);
		}
		else {
			errorHandler.setInfoNoLog(F("reading failed\r\n"));
		}

	}
}


void cmd_showi2c(int arg_cnt, char **args)
{
	if (checkManualMode()) {
		i2cInOut::I2C_scan();
	}
}

void cmd_showRange(int arg_cnt, char **args)
{
	rangeSensor.flagShowRange = true;
}

void cmd_showBumper(int arg_cnt, char **args)
{
	bumperSensor.flagShowBumper = true;
}

void cmd_showChargeSystem(int arg_cnt, char **args)
{
	chargeSystem.flagShowChargeSystem = true;
}


void cmd_printError(int arg_cnt, char **args)
{
	if (checkManualMode()) {
		errorHandler.print();
	}

}

void cmd_resetError(int arg_cnt, char **args)
{
	errorHandler.resetError();
	lastTimeShowError = 0;
	motorDriver.resetFault();
	mowMotorDriver.resetFault();
}



void cmd_showMem(int arg_cnt, char **args)
{
	FreeMem();
}


void cmd_showADCPrint(int arg_cnt, char **args)
{
	ADCMan.printInfo();
}

void cmd_showADCValues(int arg_cnt, char **args)
{
	ADCMan.showValuesOnConsole = true;
}


void cmd_showGotoAreaDistance(int arg_cnt, char **args)
{
	motor.flagShowDistance = true;
}


void cmd_showWorkx(int arg_cnt, char **args)
{
	SETTB(TB_SHOW_ROTATE);
	errorHandler.setInfoNoLog(F("showWorkx\r\n"));
}



void cmd_hideShowing(int arg_cnt, char **args)
{
	clcL.flagShowSpeed = false;
	clcR.flagShowSpeed = false;
	clcL.flagShowEncoder = false;
	clcR.flagShowEncoder = false;
	mowMotorSensor.showValuesOnConsole = false;
	perimeterSensoren.showValuesOnConsole = false;
	rangeSensor.flagShowRange = false;
	bumperSensor.flagShowBumper = false;
	chargeSystem.flagShowChargeSystem = false;
	motor.flagShowDistance = false;
	CLRTB(TB_SHOW_ROTATE);
	ADCMan.showValuesOnConsole = false;
	rtc.flagShowRTCRead = false;
	clcL.myPID.flagShowPID = false;
	clcR.myPID.flagShowPID = false;
	pcL.flagShowResults = false;
	pcR.flagShowResults = false;
	clcM.flagShowSpeed = false;
	errorHandler.setInfoNoLog(F("HIDE\r\n"));

}


void cmd_setGoHomePID(int arg_cnt, char **args)
{

	trackPerimeter.Ki = cmdStr2Float(args[1]);
	errorHandler.setInfoNoLog(F("KI: %f\r\n"), trackPerimeter.Ki);

}

void cmd_setChargeRelay(int arg_cnt, char **args)
{
	int i = cmdStr2Num(args[1], 10);

	if (i == 0) {
		chargeSystem.deactivateRelay();
		errorHandler.setInfoNoLog(F("Relay disabled i: %d\r\n"), i);
	}
	else {
		chargeSystem.activateRelay();
		errorHandler.setInfoNoLog(F("Relay enabled: %d\r\n"), i);
	}
}


void cmd_setMowMode(int arg_cnt, char **args)
{
	int i = cmdStr2Num(args[1], 10);
	if (i == 0) {
		myBlackboard.flagMowMode = MOWMODE_WORKX;
		errorHandler.setInfoNoLog(F("MowMode Workx i: %d\r\n"), i);
	}
	else {
		myBlackboard.flagMowMode = MOWMODE_308;
		errorHandler.setInfoNoLog(F("MowMode 308 i: %d\r\n"), i);
	}
}



void cmd_checkBTModule(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		BluetoothConfig bt;
		bt.checkModule(true);
	}
}

void cmd_setBTBodule(int arg_cnt, char **args)
{
	if (GETTB(TB_CONTROL_MANUAL)) {
		BluetoothConfig bt;
		bt.setParams(F("Ardumower"), 1234, 115200, true);
	}
}

void cmd_setCruiseSpiral(int arg_cnt, char **args)
{
	int i = cmdStr2Num(args[1], 10);

	if (i == 0) {
		myBlackboard.flagCruiseSpiral = false;
		errorHandler.setInfoNoLog(F("CruiseSpiral disabled i: %d\r\n"), i);
	}
	else {
		myBlackboard.flagCruiseSpiral = true;
		errorHandler.setInfoNoLog(F("CruiseSpiral enabled: %d\r\n"), i);
	}
}


void cmd_setProcessingConnected(int arg_cnt, char **args)
{
	int i = cmdStr2Num(args[1], 10);

	if (i == 0) {
		CLRTB(TB_SHOW_PROCESSING_DATA);
		errorHandler.setInfoNoLog(F("Processing Disconnected\r\n"), i);
	}
	else {
		SETTB(TB_SHOW_PROCESSING_DATA);
		errorHandler.setInfoNoLog(F("Processing Connected\r\n"), i);
	}
}


void cmd_showTables(int arg_cnt, char **args)
{

	errorHandler.setInfoNoLog(F("--- FLOAT TABLE --\r\n"));

	for (int idx = 0; idx < TABLE_F_END; idx++) {
		errorHandler.setInfoNoLog(F("idx: %i value: %f\r\n"), idx, GETTF(idx));
	}

	errorHandler.setInfoNoLog(F("--- INT TABLE --\r\n"));
	for (int idx = 0; idx < TABLE_I_END; idx++) {

		errorHandler.setInfoNoLog(F("idx: %i value: %d\r\n"), idx, GETTI(idx));
	}

	errorHandler.setInfoNoLog(F("--- BOOL TABLE --\r\n"));
	for (int idx = 0; idx < TABLE_B_END; idx++) {

		errorHandler.setInfoNoLog(F("idx: %i value: %d\r\n"), idx, GETTB(idx));
	}

}

void cmd_setFTable(int arg_cnt, char **args)
{
	int idx = cmdStr2Num(args[1], 10);
	float val = cmdStr2Float(args[2]);
	if (idx >= 0 && idx < TABLE_F_END) {
		SETTF(idx, val);
	}
}

void cmd_setITable(int arg_cnt, char **args)
{
	int idx = cmdStr2Num(args[1], 10);
	int  val = cmdStr2Num(args[2], 10);
	if (idx >= 0 && idx < TABLE_I_END) {
		SETTI(idx, val);
	}
}


void cmd_setBTable(int arg_cnt, char **args)
{
	int idx = cmdStr2Num(args[1], 10);
	int val = cmdStr2Num(args[2], 10);

	if (idx >= 0 && idx < TABLE_B_END) {
		if (val == 0) {
			CLRTB(idx);
		}
		else {
			SETTB(idx);
		}
	}

	errorHandler.setInfoNoLog(F("Value of idx: %d = %d\r\n"), idx, GETTB(idx));

}

// Print "hello world" when called from the command line.
//
// Usage:
// hello
void cmd_hello(int arg_cnt, char **args)
{
	errorHandler.setInfoNoLog(F("Hello world.\r\n"));
}
// Display the contents of the args string array.
//
// Usage:
// args 12 34 56 hello gothic baby
//
// Will display the contents of the args array as a list of strings
// Output:
// Arg 0: args
// Arg 1: 12
// Arg 2: 34
// Arg 3: 56
// Arg 4: hello
// Arg 5: gothic
// Arg 6: baby
void cmd_arg_display(int arg_cnt, char **args)
{
	for (int i = 0; i < arg_cnt; i++) {
		errorHandler.setInfoNoLog(F("Arg %i: %s\r\n"), i, args[i]);
	}
}

/********************************************************************************
********************************************************************************
**  setup function for user commands
**
********************************************************************************
*********************************************************************************/


void cmd_setup()
{


	cmdAdd((char *)"hello", cmd_hello);

	cmdAdd((char *)"args", cmd_arg_display);
	cmdAdd((char *)"H", cmd_help);




	// Mode Selection
	cmdAdd((char *)"M", cmd_cntrManuel);
	cmdAdd((char *)"A", cmd_cntrAuto);
	cmdAdd((char *)"area", cmd_cntrGotoAreaX);
	cmdAdd((char *)"gohome", cmd_goHome);

	cmdAdd((char *)"rtc.show", cmd_showRTC);
	cmdAdd((char *)"rtc.find", cmd_showRTCfind);
	cmdAdd((char *)"rtc.set", cmd_setRTC);
	cmdAdd((char *)"eep.u8t", cmd_showEEPROMbyte);
	cmdAdd((char *)"eep.s32t", cmd_showEEPROM32t);
	cmdAdd((char *)"eep.f", cmd_showEEPROMfloat);
	cmdAdd((char *)"eep.set.u8t", cmd_setEEPROMbyte);
	cmdAdd((char *)"eep.set.s32t", cmd_setEEPROM32t);
	cmdAdd((char *)"eep.set.f", cmd_setEEPROMfloat);

	// closed loop control service
	//------------------------------
	cmdAdd((char *)"clc.config", cmd_clc_show_config);
	cmdAdd((char *)"clc.enc", cmd_clc_showEncoder);
	cmdAdd((char *)"clc.pidl", cmd_clc_showPIDL);
	cmdAdd((char *)"clc.pidr", cmd_clc_showPIDR);

	cmdAdd((char *)"clc.speedl", cmd_clc_showSpeedL);
	cmdAdd((char *)"clc.speedr", cmd_clc_showSpeedR);
	cmdAdd((char *)"clc.v", cmd_clc_setSpeed);
	cmdAdd((char *)"clc.s", cmd_clc_driveStop);
	cmdAdd((char *)"clc.p", cmd_clc_setKP);
	cmdAdd((char *)"clc.i", cmd_clc_setKI);
	cmdAdd((char *)"clc.d", cmd_clc_setKD);
	cmdAdd((char *)"clc.ramp", cmd_clc_setRamp);
	cmdAdd((char *)"clc.ag", cmd_clc_setAgility);

	
	cmdAdd((char *)"clc.mt", cmd_clc_motorTest);
	cmdAdd((char *)"clc.msp", cmd_clc_motorStepSpeed);
	cmdAdd((char *)"clc.msps", cmd_clc_motorFSB);

	// Mow Motor CLCM
	cmdAdd((char *)"clcm.config", cmd_clcm_show_config);
	cmdAdd((char *)"clcm.speed", cmd_clcm_showSpeed);
	cmdAdd((char *)"clcm.accel", cmd_clcm_setRamp);
	cmdAdd((char *)"clcm.limit", cmd_clcm_setSpeedLimit);

	cmdAdd((char *)"z", cmd_startMowMot);
	cmdAdd((char *)"t", cmd_stopMowMot);

	// Position Control Service
	//------------------------------
	cmdAdd((char *)"pc.config", cmd_pos_show_config);
	cmdAdd((char *)"pc.L", cmd_pos_show_resultsL);
	cmdAdd((char *)"pc.R", cmd_pos_show_resultsR);
	cmdAdd((char *)"pc.tuneup", cmd_pc_setTuneup);
	cmdAdd((char *)"pc.a", cmd_driveAngle);
	cmdAdd((char *)"pc.cm", cmd_driveCM);
	cmdAdd((char *)"pc.stop", cmd_stopPositioning);


	

	cmdAdd((char *)"show.mowsens", cmd_showMowSensor);



	cmdAdd((char *)"set.ghpid", cmd_setGoHomePID);
	cmdAdd((char *)"set.relay", cmd_setChargeRelay);
	cmdAdd((char *)"set.proc", cmd_setProcessingConnected);
	cmdAdd((char *)"set.mowmode", cmd_setMowMode);
	cmdAdd((char *)"set.spiral", cmd_setCruiseSpiral);
	cmdAdd((char *)"set.f", cmd_setFTable);
	cmdAdd((char *)"set.b", cmd_setBTable);
	cmdAdd((char *)"set.i", cmd_setITable);




	cmdAdd((char *)"turnto", cmd_turnTo);


	// show commands mode independent

	cmdAdd((char *)"show.bat", cmd_showBattery);
	cmdAdd((char *)"show.per", cmd_showPerimeter);
	cmdAdd((char *)"show.permax", cmd_showPerimeterMax);


	cmdAdd((char *)"i2c.scan", cmd_showi2c);






	cmdAdd((char *)"show.range", cmd_showRange);
	cmdAdd((char *)"show.bumper", cmd_showBumper);
	cmdAdd((char *)"show.charge", cmd_showChargeSystem);
	cmdAdd((char *)"show.mem", cmd_showMem);
	cmdAdd((char *)"show.distance", cmd_showGotoAreaDistance);
	cmdAdd((char *)"show.workx", cmd_showWorkx);
	cmdAdd((char *)"show.tables", cmd_showTables);
	cmdAdd((char *)"show.adc", cmd_showADCPrint);
	cmdAdd((char *)"show.adcv", cmd_showADCValues);
	cmdAdd((char *)"h", cmd_hideShowing);
	// Bluetooth
	cmdAdd((char *)"bt.show", cmd_checkBTModule);
	cmdAdd((char *)"bt.set", cmd_setBTBodule);
	// error handling
	cmdAdd((char *)"error", cmd_printError);
	cmdAdd((char *)"reset", cmd_resetError);
}



