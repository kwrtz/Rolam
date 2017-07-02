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

#include "i2c.h"
#include "hardware.h"
#include "cmd.h"
#include "perimeter.h"
#include "Thread.h"
#include "ThreadController.h"
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
#include "chargeSystem.h"
#include "tables.h"
#include "ui.h"
#include "printSensordata.h"


/*********************************************************************/
// Global Variables
/*********************************************************************/
unsigned long loopCounter = 0;
bool processingConnectedFlag = false;

/* Private function prototypes -----------------------------------------------*/
void loop();


TErrorHandler errorHandler;

/*********************************************************************/
// Controller Threads die in bestimmten intervallen aufgerufen werden
// ACHTUNG: In TreadController.h die Anzahl der threads einstellen falls 15 überschritten wird
/*********************************************************************/

// mow motor closed loop control - no closed loop used
TMowClosedLoopControlThread clcM;
// drive motor left closed loop control
TClosedLoopControlThread clcL;
// drive motor rigth closed loop control
TClosedLoopControlThread clcR;
//drive motor left position control
TPositionControl pcL;
//drive motor right position control
TPositionControl pcR;
// Motorensteuerung Interface. KEIN THREAD. Wird verwendet um clcX und pcX zu steuern.
TMotorInterface motor;
// Verarbeitet Ein-/Ausgabe über Serial line console oder bluetooth. in hardware.cpp die debug zuweisung ändern.
Thread cmd;
// Daten von Perimetersensoren vom 446RE
TPerimeterThread perimeterSensoren;
// Messung der Batteriespannung
TbatterieSensor batterieSensor;
// Messung des Mähmotorstroms
TMowMotorSensor mowMotorSensor;
// SRF08 Range Sensor Messung der Entfernung
TrangeSensor rangeSensor;
// Bumper Sensor
TbumperSensor bumperSensor;
// Charge System
TchargeSystem chargeSystem;
// Print Sensordata for processing
Thread processingSensorData;
// Instantiate a new ThreadController
ThreadController controller = ThreadController(); // Thread die vor manuellen mode laufen müssen



/*********************************************************************/
// Behaviour Objects die auf die Threads oben zugreifen
/*********************************************************************/

Blackboard myBlackboard(motor, perimeterSensoren, mowMotorSensor, rangeSensor, bumperSensor, batterieSensor, encoderL, encoderR, chargeSystem);
TBehaviour myBehaviour(myBlackboard);


/*********************************************************************/
// Module Variables
/*********************************************************************/
bool _controlManuel = true;
unsigned long lastTimeShowError = 0;


void setup()
{

	//---------------------------------
	// Initialize interface to hardware
	//---------------------------------
	hardwareSetup();
	delay(2000); // wait for motordriver and SRF08 ready

				 // Set start configuration
	tables_init();


	//---------------------------------
	// Threads konfigurieren
	// Motor-/Positionthreads laufen in 20ms bzw 100ms Takt.
	// Alle andern in anderen Intervallen.
	//---------------------------------
	clcM.setup(1);  // Mow Motor 1 of sabertooth
	clcM.setInterval(198);

	clcL.setup(1, &encoderL);  // Motor 1 of sabertooth
	clcL.setInterval(20);
	clcR.setup(2, &encoderR);  // Motor 2 of sabertooth
	clcR.setInterval(20);

	pcL.setup(&clcL, &encoderL);  // Position control left
	pcL.setInterval(100);
	pcR.setup(&clcR, &encoderR);  // Position control right
	pcR.setInterval(100);

	motor.setup(&clcM, &clcL, &clcR, &pcL, &pcR);

	//---------------------------------
	cmdInit();
	cmd.onRun(cmdPoll);
	cmd.setInterval(116);
	//---------------------------------
	perimeterSensoren.setup();
	perimeterSensoren.setInterval(0); // immer aufrufen und checken ob ein neues byte empfangen wurde
									  //---------------------------------
	batterieSensor.setup();
	batterieSensor.setInterval(1974);
	//---------------------------------
	mowMotorSensor.setup();
	mowMotorSensor.setInterval(100);
	//---------------------------------
	rangeSensor.setup();
	rangeSensor.setInterval(137);
	rangeSensor.enabled = false;
	//---------------------------------
	bumperSensor.setup();
	bumperSensor.setInterval(15);
	//---------------------------------
	chargeSystem.setup();
	chargeSystem.setInterval(53);
	//---------------------------------
	processingSensorData.onRun(printSensordata);
	processingSensorData.setInterval(1007);


	// ACHTUNG: In TreadController.h die Anzahl der threads einstellen falls 15 überschritten wird
	controller.add(&clcM);
	controller.add(&clcL);
	controller.add(&clcR);
	controller.add(&pcL);
	controller.add(&pcR);
	controller.add(&cmd);
	controller.add(&perimeterSensoren);
	controller.add(&batterieSensor);
	controller.add(&mowMotorSensor);
	controller.add(&rangeSensor);
	controller.add(&bumperSensor);
	controller.add(&chargeSystem);
	controller.add(&processingSensorData);

	//---------------------------------
	// Behaviour Objects konfigurieren
	//---------------------------------

	myBehaviour.setup();

	//bMow.setup(&motor, &perimeterSensoren, &mowMotorSensor, &errorHandler, &rangeSensor, &bumperSensor);
	//bFollowLine.setup(&motor, &perimeterSensoren, &errorHandler);
	//arbitrator.setup(&motor, &perimeterSensoren, &batterieSensor, &mowMotorSensor, &errorHandler, &bMow);

	//---------------------------------
	// Userinterface setup
	//---------------------------------
	cmd_setup();


	motor.stopAllMotors();
	// Check if in charging station
	for (int i = 0; i<10; i++) { // Read charging voltage
		chargeSystem.run();
	}
	if (chargeSystem.isInChargingStation()) {
		myBlackboard.setBehaviour(BH_CHARGING);
		_controlManuel = false;
	}

}

void executeLoop() //wird von ui.cpp verwendet wenn Hilfe ausgegeben wird
{
	loop();
}

void loop()
{
	debug.run();
	perRX.run();

	loopCounter++;

	if (errorHandler.isErrorActive()) {
		_controlManuel = true;
		////arbitrator.SetState(STARB_OFF);
		motor.stopAllMotors();

		if (lastTimeShowError == 0) {
			lastTimeShowError = millis();
		}

		if (millis() - lastTimeShowError > 2000) {
			lastTimeShowError = millis();
			doMyLED = !doMyLED;
			errorHandler.printError();
		}
	}

	controller.run();


	//clcR.testEncoder();

	// Auswerten ob user manuellen mode gesetzt hat und kommando ausführen
	// Auszuwertende variablen sind in Klasse TSerialEventThread definiert
	if (_controlManuel) {

	}
	else {

		// Watchdog functions
		//--------------------------------------------------

		//if (perimeterSensoren.signalTimedOut() && perimeterSensoren.magnetudeR != 0 && perimeterSensoren.magnetudeL != 0) {
		//    errorHandler.setError("perimeter signal timed out");
		//    return;
		//}

		//--------------------------------------------------

		myBehaviour.loop();
	}
}



