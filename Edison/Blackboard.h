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

#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "NodeStack.h"

#include "hardware.h"
#include "global.h"
#include "batterySensor.h"
#include "motor.h"
#include "perimeter.h"
#include "errorhandler.h"
#include "rangeSensor.h"
#include "bumperSensor.h"
#include "mowmotorSensor.h"
#include "chargeSystem.h"



    
enum enuDriveDirection {
    DD_FORWARD = 0,
    DD_FORWARD_INSIDE = 1, //Darf eigentlich nicht vorkommen wenn ich prüfe, dass Spule hinten rausfährt beim zurückfahren
    DD_OVERRUN = 2,
    DD_REVERSE_ESC_OBST  = 3,
    DD_REVERSE_INSIDE = 4,
    DD_ROTATECW = 5,
    DD_ROTATECC = 6,
    DD_REVERSE_LINE_FOLLOW = 7,
    DD_FORWARD20 = 8,
    DD_SPIRAL_CW = 9
};

extern const char* enuDriveDirectionString[];


enum enuFlagCoilsOutside  {CO_NONE, CO_RIGHT, CO_LEFT, CO_BOTH };
enum enuFlagPeriStateChanged  {PSC_NONE, PSC_IO, PSC_OI};
enum enuFlagForceRotateDirection  {FRD_NONE, FRD_CW, FRD_CC};
enum enuFlagMowMode  {MOWMODE_308, MOWMODE_WORKX };

enum enuFlagEscabeObstacleConFlag  {FEO_NONE, FEO_BACK180, FEO_FWD20, FEO_ROTCC, FEO_ROTCW, FEO_BACKINSIDE, FEO_ROT}; // Condition which algortihm will be used by mselEscabeObstacle
extern const char* enuFlagEscabeObstacleConFlagString[];

enum enuBehaviour  {BH_NONE, BH_MOW, BH_PERITRACK, BH_CHARGING, BH_GOTOAREA, BH_FINDPERIMETER};


class Node;


// We will use a Blackboard object. This object will be created by the user and will be propagated to the nodes during the Blackboard through the tick function.
class Blackboard
{
public:
    Blackboard(TMotorInterface &_motor, TPerimeterThread &_perimeterSenoren, TMowMotorSensor& _mowMotorSensor, TrangeSensor &_rangeSensor,
               TbumperSensor &_bumperSensor,  TbatterieSensor& _batterieSensor, CRotaryEncoder &_encoderL, CRotaryEncoder &_encoderR, TchargeSystem &_chargeSystem):
        motor(_motor),
        perimeterSensoren(_perimeterSenoren),
        rangeSensor(_rangeSensor),
        bumperSensor(_bumperSensor),
        mowMotorSensor(_mowMotorSensor),
        batterieSensor(_batterieSensor),
        encoderL(_encoderL),
        encoderR(_encoderR),
        chargeSystem(_chargeSystem)
    {}


    static const int CRUISE_SPEED_MEDIUM = 70;
    static const int CRUISE_SPEED_LOW = 50;
    static const int CRUISE_SPEED_OBSTACLE = 40;  // Lowest Speed to drive
    static const int CRUISE_SPEED_HIGH = 92;
    static const int CRUISE_ROTATE_LOW = 15;
    static const int LINEFOLLOW_SPEED_LOW = 50;
    static const int LINEFOLLOW_SPEED_HIGH = 50;
    static const int SHORTWAYCOUNT  = 3; //3


    enuFlagCoilsOutside   flagCoilFirstOutside;
    enuFlagCoilsOutside   flagCoilOutsideAfterOverrun;
    enuFlagPeriStateChanged  flagPerimeterStateChanged;
    enuFlagForceRotateDirection flagForceRotateDirection;
    enuFlagEscabeObstacleConFlag flagEscabeObstacleConFlag;
    enuFlagMowMode   flagMowMode;

    uint16_t shortWayCounter;
    bool flagBumperInsidePerActivated ;
    bool flagBumperOutsidePerActivated;
    bool flagCruiseSpiral;
    //bool flagPerimeterActivated;
    
    int flagForceSmallRotAngle; // Anzahl, wie haeufig kleiner Winkel gedreht werden soll 


    bool flagEnableMowing;
    bool flagEnablePerimetertracking;
    bool flagEnableCharging;
    bool flagEnableGotoAreaX;
    bool flagEnableFindPerimeter;
    
    bool flagGotoAreaXFirstCall;

    bool flagGoHome;

    int cruiseSpeed;
    unsigned long timeCruiseSpeedSet;

    enuDriveDirection driveDirection;

    int arcRotateXArc;

    long areaTargetDistanceInMeter;
   
    unsigned long lastTimeSpiralStarted;
   

    TMotorInterface& motor;
    TPerimeterThread& perimeterSensoren;
    TrangeSensor& rangeSensor;
    TbumperSensor& bumperSensor;
    TMowMotorSensor& mowMotorSensor;
    TbatterieSensor& batterieSensor;
    CRotaryEncoder& encoderL;
    CRotaryEncoder& encoderR;
    TchargeSystem& chargeSystem;

    // Wird von BehaviourTree funktionen verwendet und muss immer vorhanden sein.
    Node* lastNodeLastRun;
    Node* lastNodeCurrentRun;
    NodeStack runningNodes;
    //

    void setBehaviour(enuBehaviour b );


};


#endif