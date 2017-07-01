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

#include "Blackboard.h"

/*
enum enuDriveDirection {
    DD_FORWARD = 0,
    DD_FORWARD_INSIDE = 1,
    DD_OVERRUN = 2,
    DD_REVERSE_ESC_OBST  = 3,
    DD_REVERSE_INSIDE = 4,
    DD_REVERSE_INSIDE_CURVE_CW = 5,
    DD_REVERSE_INSIDE_CURVE_CC = 6 ,
    DD_ROTATECW = 7,
    DD_ROTATECC = 8,
    DD_REVERSE_LINE_FOLLOW = 9,
};
*/

const char* enuDriveDirectionString[] = { "DD_FORWARD",
                                        "DD_FORWARD_INSIDE",
                                        "DD_OVERRUN",
                                        "DD_REVERSE_ESC_OBST",
                                        "DD_REVERSE_INSIDE",
                                        "DD_ROTATECW",
                                        "DD_ROTATECC",
                                        "DD_REVERSE_LINE_FOLLOW",
                                        "DD_FORWARD20",
                                        "DD_SPIRAL_CW",
                                        "UNKNOWN4",
                                        "UNKNOWN5",
                                        "UNKNOWN6"
                                        };

const char* enuFlagEscabeObstacleConFlagString[] = {    "FEO_NONE",
        "FEO_BACK180",
        "FEO_FWD20",
        "FEO_ROTCC",
        "FEO_ROTCW",
        "FEO_BACKINSIDE",
        "FEO_ROT",
        "UNKNOWN3",
        "UNKNOWN4",
        "UNKNOWN4",
        "UNKNOWN4",
        "UNKNOWN5"
                                                   };


void Blackboard::setBehaviour(enuBehaviour b )
{

    flagEnableMowing = false;
    flagEnablePerimetertracking = false;
    flagEnableCharging = false;
    flagEnableGotoAreaX = false;
    flagEnableFindPerimeter = false;
    motor.enableDefaultRamping();


    switch(b) {
        case BH_GOTOAREA:
            flagEnableGotoAreaX = true;
            flagGotoAreaXFirstCall = true;
            rangeSensor.enabled = false;
            chargeSystem.deactivateRelay();  //Muss hier auch abgeschaltet werden, falls user dieses ueber das UI einschaltet.
            //motor.mowMotStop(); // Will be started in BH_MOW behaviour tree.
            motor.stopAllMotors(); // Nur zur Sicherheit, falls diese gerade laufen.
            motor.startDistanceMeasurementAreax();
            errorHandler.setInfo("!04,SET BEHAV -> BH_GOTOAREA\r\n");
            break;
        case BH_CHARGING:
            flagEnableCharging = true;
            rangeSensor.enabled = false;
            motor.mowMotStop();
            errorHandler.setInfo("!04,SET BEHAV -> BH_CHARGING\r\n");
            break;
        case BH_PERITRACK:
            flagEnablePerimetertracking = true;
            rangeSensor.enabled = false;
            chargeSystem.deactivateRelay();
            //motor.mowMotStop();
            errorHandler.setInfo("!04,SET BEHAV -> BH_PERITRACK\r\n");
            break;
        case BH_FINDPERIMETER:
            flagEnableFindPerimeter = true;
            rangeSensor.enabled = true;
            chargeSystem.deactivateRelay();
            //motor.mowMotStop();
            errorHandler.setInfo("!04,SET BEHAV -> BH_FINDPERIMETER\r\n");
            break;
        case BH_MOW:
            flagEnableMowing = true;
            rangeSensor.enabled = true;
            chargeSystem.deactivateRelay();
            motor.resetEncoderCounter();
            motor.startDistanceMeasurementWorkx();
            errorHandler.setInfo("!04,SET BEHAV -> BH_MOW\r\n");
            break;
        case BH_NONE:
            rangeSensor.enabled = false;
            chargeSystem.deactivateRelay();
            motor.mowMotStop();
            errorHandler.setInfo("!04,SET BEHAV -> BH_NONE\r\n");
            break;
        default:
            errorHandler.setError("!04,setBehaviour unbekanntes Behaviour\r\n");
    }
}