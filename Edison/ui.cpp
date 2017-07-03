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

extern bool processingConnectedFlag;
extern void executeLoop();
extern bool _controlManuel;
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
// Messung des Mähmotorstroms
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

extern void FreeMem(void);


/********************************************************************************
********************************************************************************
**  functions for user commands
**  each function represents one user command
********************************************************************************
*********************************************************************************/

void cmd_help(int arg_cnt, char **args)
{
    unsigned long wait;

	errorHandler.setInfoNoLog(F("\Edison interface emulator\r\n"));
    errorHandler.setInfoNoLog(F("Available debug commands: (lines end with CRLF or '\\r')\r\n"));
    errorHandler.setInfoNoLog(F("H will print this help message again\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("hello print hello message\r\n"));
    errorHandler.setInfoNoLog(F("args,1,2,3 show args 1 2 3\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();



    errorHandler.setInfoNoLog(F("\r\nTUNE PARAMETER\r\n"));
    errorHandler.setInfoNoLog(F("set.p,123.34  //sets drive motors proportional term\r\n"));
    errorHandler.setInfoNoLog(F("set.i,123.34 //sets drive motors integral term\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("set.d,123.34 //sets drive motors derivative term\r\n"));
    errorHandler.setInfoNoLog(F("set.poskp,123.34 //sets position proportional term\r\n"));
    errorHandler.setInfoNoLog(F("set.ghpid,0.1 0.2 0.3 //set line follower pid kp ki kd\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("set.twf,1.49 //set turn winkel faktor to 1.49\r\n"));
    errorHandler.setInfoNoLog(F("set.relay,1/0 //turn relay on/off\r\n"));
    errorHandler.setInfoNoLog(F("set.proc,1/0  //turn output for processing on/off\r\n"));

    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("set.mowmode,0/1  0=Workx 1=308\r\n"));
    errorHandler.setInfoNoLog(F("set.spiral,0/1  0=Off 1=On\r\n"));
	errorHandler.setInfoNoLog(F("set.f,1,10 set f table value idx=1 value=10\r\n"));
	wait = millis();
	while (millis() - wait < 200) executeLoop();

	errorHandler.setInfoNoLog(F("set.i,1,10 set i table value idx=1 value=10\r\n"));
	errorHandler.setInfoNoLog(F("set.b,1,0/1 set b table value idx=1 value = 0 or 1\r\n"));
    errorHandler.setInfoNoLog(F("\n\r\nMODE SELECTION\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("A //automatic control over actuators - standard\r\n"));
    errorHandler.setInfoNoLog(F("M //manuel control over actuators\r\n"));
    errorHandler.setInfoNoLog(F("area,12 //drive 12m at perimeter and begin mowing\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("gohome //drive to docking station. Call again to deactivate\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();


    errorHandler.setInfoNoLog(F("\n\r\nMANUAL MODE COMMANDS\r\n"));
    errorHandler.setInfoNoLog(F("v,30 //drives motors with speed of 30%\r\n"));
    errorHandler.setInfoNoLog(F("     //Value: -100% to 100%\r\n"));

    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("s //stop drive motors\r\n"));
    errorHandler.setInfoNoLog(F("drivew,60,30 //drives 60 degrees with speed 30\r\n"));
    errorHandler.setInfoNoLog(F("drivecm,60,30 //drives 60 cm with speed 30\r\n"));
    errorHandler.setInfoNoLog(F("              //negative drives backward\r\n"));

    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("turnto,60,30  //turn 60 degrees right with speed 30\r\n"));
    errorHandler.setInfoNoLog(F("              //negative turns left\r\n"));

    errorHandler.setInfoNoLog(F("sp //stop Positioning\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("z //Mow motor start\r\n"));
    errorHandler.setInfoNoLog(F("t //mow motor stop\r\n"));
    errorHandler.setInfoNoLog(F("\n\r\nSHOW COMMANDS\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("show.pid //shows dirvemotor pid values\r\n"));
    errorHandler.setInfoNoLog(F("show.bat //shows battery voltage\r\n"));

    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("show.per //shows perimeter sensor values\r\n"));
    errorHandler.setInfoNoLog(F("show.permax //shows maximum perimeter value\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("show.micros //shows return value of micros()\r\n"));
    errorHandler.setInfoNoLog(F("show.mowsens //show mowsensor\r\n"));
    errorHandler.setInfoNoLog(F("show.speedl //show speed left\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("show.speedr //show speed right\r\n"));
    errorHandler.setInfoNoLog(F("show.range //show rangesensor values\r\n"));
    errorHandler.setInfoNoLog(F("show.bumper //show bumper sensor if active\r\n"));

    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("show.charge //shows charge sensors\r\n"));
    errorHandler.setInfoNoLog(F("show.mem //shows free memory\r\n"));
    errorHandler.setInfoNoLog(F("show.distance //shows distance while drving to areaX\r\n"));

    wait = millis();
    while (millis() - wait < 200) executeLoop();
    errorHandler.setInfoNoLog(F("show.workx //shows workx rotate values\r\n"));
	errorHandler.setInfoNoLog(F("show.tables //shows config tables\r\n"));
	

    errorHandler.setInfoNoLog(F("h //hide showing\r\n"));

    wait = millis();
    while (millis() - wait < 200) executeLoop();

    errorHandler.setInfoNoLog(F("\n\r\nERROR HANDLING\r\n"));
    errorHandler.setInfoNoLog(F("error //shows errormessage\r\n"));
    errorHandler.setInfoNoLog(F("reset //reset error\r\n"));
    wait = millis();
    while (millis() - wait < 200) executeLoop();
}


void cmd_goHome(int arg_cnt, char **args)
{
    myBlackboard.flagGoHome = true;
}


void cmd_setKP(int arg_cnt, char **args)
{
    float val = cmdStr2Float(args[1]);
    clcL.myPID.SetKp(val);
    clcR.myPID.SetKp(val);
}

void cmd_setKI(int arg_cnt, char **args)
{
    float val = cmdStr2Float(args[1]);
    clcL.myPID.SetKi(val);
    clcR.myPID.SetKi(val);
}

void cmd_setKD(int arg_cnt, char **args)
{
    float val = cmdStr2Float(args[1]);
    clcL.myPID.SetKd(val);
    clcR.myPID.SetKd(val);
}

void cmd_showpid(int arg_cnt, char **args)
{
    errorHandler.setInfoNoLog(F( "KP: %f KI: %f  KD: %f\r\n"),clcL.myPID.GetKp(), clcL.myPID.GetKi(),clcL.myPID.GetKd()) ;
}

void cmd_setSpeed(int arg_cnt, char **args)
{
    float val = cmdStr2Float(args[1]);
    clcL.setSpeed(val);
    clcR.setSpeed(val);
}

void cmd_driveStop(int arg_cnt, char **args)
{
    if (_controlManuel) {
        //motor.stop();
        clcL.stop();
        clcR.stop();
    }
}

void cmd_driveW(int arg_cnt, char **args)
{
    if (_controlManuel) {
        float winkel = cmdStr2Float(args[1]);
        float speed = cmdStr2Float(args[2]);
        motor.rotateAngle(winkel,speed);
    }
}

void cmd_driveCM(int arg_cnt, char **args)
{
    if (_controlManuel) {
        float cm = cmdStr2Float(args[1]);
        float speed = cmdStr2Float(args[2]);
        motor.rotateCM(cm,speed);
    }
}

void cmd_turnTo(int arg_cnt, char **args)
{
    if (_controlManuel) {
        float winkel = cmdStr2Float(args[1]);
        float speed = cmdStr2Float(args[2]);
        motor.turnTo(winkel, speed);
    }
}

void cmd_stopPositioning(int arg_cnt, char **args)
{
    if (_controlManuel) {
        motor.stop();
    }
}


void cmd_startMowMot(int arg_cnt, char **args)
{
    if (_controlManuel) {
        motor.mowMotStart();
        motor.M->uiMotorDisabled = false;
        motor.M->motorDisabled = false;
    } else {
        motor.M->uiMotorDisabled = false;
    }
}

void cmd_stopMowMot(int arg_cnt, char **args)
{
    if (_controlManuel) {
        motor.mowMotStop();
    } else {
        motor.M->uiMotorDisabled = true;
    }
}

void cmd_cntrManuel(int arg_cnt, char **args)
{
    _controlManuel = true;
    motor.stopAllMotors();
    myBehaviour.reset();

}

void cmd_cntrAuto(int arg_cnt, char **args)
{
    if(myBlackboard.flagEnableMowing == false) {
        _controlManuel = false;
        myBehaviour.reset();
        myBlackboard.setBehaviour(BH_MOW);
    }

}

void cmd_cntrGotoAreaX(int arg_cnt, char **args)
{
    _controlManuel = false;
    myBehaviour.reset();
    long distance = cmdStr2Num(args[1],10);
    myBlackboard.areaTargetDistanceInMeter = distance;
    myBlackboard.setBehaviour(BH_GOTOAREA);
    errorHandler.setInfoNoLog(F( "Drive to area: %l\r\n"),distance);

}

void cmd_setPosKP(int arg_cnt, char **args)
{
    float val = cmdStr2Float(args[1]);
    pcL.SetPosKp(val);
    pcR.SetPosKp(val);
}

void cmd_showBattery(int arg_cnt, char **args)
{
    errorHandler.setInfoNoLog(F( "Battery Voltage: %f sensorValue: %f\r\n"),batterieSensor.voltage, batterieSensor.sensorValue);
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
    errorHandler.setInfoNoLog(F( "magMax:   %ld\r\n"),perimeterSensoren.magMax);
}


void cmd_showMicros(int arg_cnt, char **args)
{
    //errorHandler.setInfoNoLog(F( "micros():   %lu\r\n",micros());
    errorHandler.setInfoNoLog(F( "millis():   %lu\r\n"),millis());
    //errorHandler.setInfoNoLog(F( "micros64(): %llu\r\n",micros64());
    //errorHandler.setInfoNoLog(F( "millis64(): %llu\r\n",millis64());
    //errorHandler.setInfoNoLog(F( "Sekunden(): %llu\r\n",micros64()/1000000ULL);
    //errorHandler.setInfoNoLog(F( "Minuten():  %llu\r\n",micros64()/1000000ULL/60ULL);
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
    if( _controlManuel != true ) {
        errorHandler.setInfoNoLog(F("!03,NEED TO BE IN MANUAL MODE!\r\n"));
        return;
    }

    errorHandler.print();
}

void cmd_resetError(int arg_cnt, char **args)
{
    errorHandler.resetError();
    lastTimeShowError = 0;
}

void cmd_setTurnWinkelFaktor(int arg_cnt, char **args)
{
    motor.turnWinkelFaktor = cmdStr2Float(args[1]);
}


void cmd_showSpeedL(int arg_cnt, char **args)
{
    clcL.flagShowSpeed= true;
}

void cmd_showSpeedR(int arg_cnt, char **args)
{
    clcR.flagShowSpeed = true;
}



void cmd_showMem(int arg_cnt, char **args)
{
	FreeMem();
}

void cmd_showGotoAreaDistance(int arg_cnt, char **args)
{
    motor.flagShowDistance = true;
}


void cmd_showWorkx(int arg_cnt, char **args)
{
    SETTB(TB_SHOW_ROTATE, 1);
    errorHandler.setInfoNoLog(F( "showWorkx\r\n"));
}



void cmd_hideShowing(int arg_cnt, char **args)
{
    clcL.flagShowSpeed = false;
    clcR.flagShowSpeed = false;
    mowMotorSensor.showValuesOnConsole = false;
    perimeterSensoren.showValuesOnConsole = false;
    rangeSensor.flagShowRange = false;
    bumperSensor.flagShowBumper = false;
    chargeSystem.flagShowChargeSystem = false;
    motor.flagShowDistance = false;
    SETTB(TB_SHOW_ROTATE, 0);

    errorHandler.setInfoNoLog(F( "HIDE\r\n"));
}


void cmd_setGoHomePID(int arg_cnt, char **args)
{

    trackPerimeter.Ki = cmdStr2Float(args[1]);
    errorHandler.setInfoNoLog(F( "KI: %f\r\n"),trackPerimeter.Ki) ;

}

void cmd_setChargeRelay(int arg_cnt, char **args)
{
    int i = cmdStr2Num(args[1],10);

    if( i == 0) {
        chargeSystem.deactivateRelay();
        errorHandler.setInfoNoLog(F( "Relay disabled i: %d\r\n"),i) ;
    } else {
        chargeSystem.activateRelay();
        errorHandler.setInfoNoLog(F( "Relay enabled: %d\r\n"),i) ;
    }
}


void cmd_setMowMode(int arg_cnt, char **args)
{
    int i = cmdStr2Num(args[1],10);
    if( i == 0) {
        myBlackboard.flagMowMode = MOWMODE_WORKX ;
        errorHandler.setInfoNoLog(F( "MowMode Workx i: %d\r\n"),i) ;
    } else {
        myBlackboard.flagMowMode =  MOWMODE_308;
        errorHandler.setInfoNoLog(F( "MowMode 308 i: %d\r\n"),i) ;
    }
}


void cmd_setCruiseSpiral(int arg_cnt, char **args)
{
    int i = cmdStr2Num(args[1],10);

    if( i == 0) {
        myBlackboard.flagCruiseSpiral = false;
        errorHandler.setInfoNoLog(F( "CruiseSpiral disabled i: %d\r\n"),i) ;
    } else {
        myBlackboard.flagCruiseSpiral = true;
        errorHandler.setInfoNoLog(F( "CruiseSpiral enabled: %d\r\n"),i) ;
    }
}


void cmd_setProcessingConnected(int arg_cnt, char **args)
{
    int i = cmdStr2Num(args[1],10);

    if( i == 0) {
        processingConnectedFlag = false;
        errorHandler.setInfoNoLog(F( "Processing Disconnected\r\n"),i) ;
    } else {
        processingConnectedFlag = true;
		errorHandler.setInfoNoLog(F( "Processing Connected\r\n"),i) ;
    }
}


void cmd_showTables(int arg_cnt, char **args)
{

	errorHandler.setInfoNoLog(F("--- FLOAT TABLE --\r\n"));

	for (int idx = 0; idx < TABLE_F_END; idx++) {
		errorHandler.setInfoNoLog(F("idx: %i value: %f:\r\n"), idx, GETTF(idx));
	}

	errorHandler.setInfoNoLog(F("--- INT TABLE --\r\n"));
	for (int idx = 0; idx < TABLE_I_END; idx++) {

		errorHandler.setInfoNoLog(F("idx: %i value: %d:\r\n"), idx, GETTI(idx));
	}

	errorHandler.setInfoNoLog(F("--- BOOL TABLE --\r\n"));
	for (int idx = 0; idx < TABLE_B_END; idx++) {

		errorHandler.setInfoNoLog(F("idx: %i value: %d:\r\n"), idx, GETTB(idx));
	}
	
}

void cmd_setFTable(int arg_cnt, char **args)
{   
	int idx = cmdStr2Num(args[1],10);
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
			SETTB(idx, false);
		}
		else {
			SETTB(idx, true);
		}
	}
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
		errorHandler.setInfoNoLog(F("Arg %i: %s\r\n"),i, args[i]);
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

    // Comands
    cmdAdd((char *)"gohome", cmd_goHome);


    // tune parameter
    cmdAdd((char *)"set.p", cmd_setKP);
    cmdAdd((char *)"set.i", cmd_setKI);
    cmdAdd((char *)"set.d", cmd_setKD);
    cmdAdd((char *)"set.poskp", cmd_setPosKP);
    cmdAdd((char *)"set.ghpid", cmd_setGoHomePID);
    cmdAdd((char *)"set.twf", cmd_setTurnWinkelFaktor);

    cmdAdd((char *)"set.relay", cmd_setChargeRelay);

    cmdAdd((char *)"set.proc", cmd_setProcessingConnected);

    cmdAdd((char *)"set.mowmode", cmd_setMowMode);

    cmdAdd((char *)"set.spiral", cmd_setCruiseSpiral);

	cmdAdd((char *)"set.f", cmd_setFTable);
	cmdAdd((char *)"set.b", cmd_setBTable);
	cmdAdd((char *)"set.i", cmd_setITable);


	





    // Mode Selection
    cmdAdd((char *)"M", cmd_cntrManuel);
    cmdAdd((char *)"A", cmd_cntrAuto);
    cmdAdd((char *)"area", cmd_cntrGotoAreaX);




    // manual mode commands
    cmdAdd((char *)"v", cmd_setSpeed);
    cmdAdd((char *)"s", cmd_driveStop);
    cmdAdd((char *)"drivew", cmd_driveW);
    cmdAdd((char *)"drivecm", cmd_driveCM);
    cmdAdd((char *)"turnto", cmd_turnTo);
    cmdAdd((char *)"sp", cmd_stopPositioning);
    cmdAdd((char *)"z", cmd_startMowMot);
    cmdAdd((char *)"t", cmd_stopMowMot);

    // show commands mode independent
    cmdAdd((char *)"show.pid", cmd_showpid);
    cmdAdd((char *)"show.bat", cmd_showBattery);
    cmdAdd((char *)"show.per", cmd_showPerimeter);
    cmdAdd((char *)"show.permax", cmd_showPerimeterMax);

    cmdAdd((char *)"show.micros", cmd_showMicros);
    cmdAdd((char *)"show.mowsens", cmd_showMowSensor);
    cmdAdd((char *)"show.speedl", cmd_showSpeedL);
    cmdAdd((char *)"show.speedr", cmd_showSpeedR);
    cmdAdd((char *)"show.range", cmd_showRange);
    cmdAdd((char *)"show.bumper", cmd_showBumper);
    cmdAdd((char *)"show.charge", cmd_showChargeSystem);
    cmdAdd((char *)"show.mem", cmd_showMem);
    cmdAdd((char *)"show.distance", cmd_showGotoAreaDistance);
    cmdAdd((char *)"show.workx", cmd_showWorkx);
	cmdAdd((char *)"show.tables", cmd_showTables);

	

    cmdAdd((char *)"h", cmd_hideShowing);



    // error handling
    cmdAdd((char *)"error", cmd_printError);
    cmdAdd((char *)"reset", cmd_resetError);

}