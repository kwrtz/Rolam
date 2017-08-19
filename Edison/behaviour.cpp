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
#include "bConditions.h"
#include "bDecorators.h"
#include "behaviour.h"
#include "bCruise.h"
#include "bFreePerimeter.h"
#include "bPerOutside.h"
#include "bFreeBumper.h"
#include "bEscapeObstacle.h"
#include "bPerimeterTracking.h"
#include "bCharging.h"
#include "bGotoAreaX.h"
#include "bCheck2.h"
#include "bEscapeOOP.h"


// ************************************
// Conditions Behaviour
//*************************************


TdnMowModeWorkx dnMowModeWorkx;
TdnMowMode308 dnMowMode308;

TConBackCoilOutAndDDReverse conBackCoilOutAndDDReverse;

TConFEO_BACK180 conFEO_BACK180;
TConFEO_BACK180_BCO conFEO_BACK180_BCO;
TConFEO_FWD20   conFEO_FWD20;
TConFEO_ROTCC   conFEO_ROTCC;
TConFEO_ROTCW   conFEO_ROTCW;
TConFEO_ROT     conFEO_ROT;
TConFEO_BACKINSIDE conFEO_BACKINSIDE;


TConWasDirectionForward conWasDirectionForward;
TConWasDirectionForward20 conWasDirectionForward20;
TConWasDirectionForwardInside conWasDirectionForwardInside;
TConWasDirectionReverseObstacle conWasDirectionReverseObstacle ;

TConWasDirectionOverrun conWasDirectionOverrun;

TConWasDirectionReverseInside conWasDirectionReverseInside;

TConWasDirectionRotateCC conWasDirectionRotateCC;
TConWasDirectionRotateCW conWasDirectionRotateCW;


TConBumperActive conBumperActive;
TConPerOutside conPerOutside;

//TconStopOvershootLeft conStopOvershootLeft;
//TconStopOvershootRight conStopOvershootRight;
TConBatLow conBatLow;

TConRightCoilOutside conRightCoilOutside;
TConLeftCoilOutside  conLeftCoilOutside;

TConInDockingStation conInDockingStation;

TconAreaReached conAreaReached;

// ************************************
// Decorator Nodes Behaviour
//*************************************

TdnCruiseSpiral dnCruiseSpiral;
TdnMowing dnMowing;
TdnPermeterTracking dnPermeterTracking;
TdnBumpPeriActivated dnBumpPeriActivated;
TdnCharging dnCharging;
TdnGotoAreaX dnGotoAreaX;
TdnFindPerimeter dnFindPerimeter;

WaitDecorator dnWaitDockingStation;
WaitDecorator dnWaitGotoArea;

// ************************************
// Escape Obstacle Outside Perimeter
//*************************************

TsetDD_FORWARD setDD_FORWARD;
Sequence seqEscOOPRot;
Sequence seqEscOOPRotCC;
Sequence seqEscOOPRotCW;


MemSequence mseqEscOOPReverse;
MemSelector mselEscObstacleOutside;

// ************************************
// Free Bumper Behaviour
//*************************************

TselEscapeAlgorithm selEscapeAlgorithm;
THardstop hardstop;
TFreeBumper freeBumper;
MemSequence mseqBumperActive;


// ************************************
// Free Perimeter Behaviour
//*************************************

MemSequence mseqPerBackCoilOutside;

TSetflagForceSmallRotAngle setFlagForceSmallRotAngle;
MemSequence mseqPerimeterForward20;

MemSequence mseqPerimeterForwardInside;

TSetArc90CW setArc90CW;
TSetArc90CC setArc90CC;
TRotateX    rotateX;
MemSequence mseqPRRCO;
MemSequence mseqPRLCO;
MemSelector mselPerimeterReverse;




TPerRotateInsideCW perRotateInsideCW;
TPerRotateInsideCC perRotateInsideCC;
TMotorStop motorStop;
MemSequence mseqPerimeterRotCC;
MemSequence mseqPerimeterRotCW;

TOverRun overRun;

TRotateBackCoilInside2 rotateBackCoilInside2;
TRotateBackCoilInside rotateBackCoilInside;
MemSequence mseqReverseInside;


TReverseInside reverseInside;
TReverseFurtherInside reverseFurtherInside;

TForwardInside forwardInside;


MemSequence mseqPerimeterRevInside;
MemSequence mseqPerimeterOverrun;


MemSequence mseqPerimeterReverse;
MemSequence mseqPerimeterForward;
MemSelector mselPerimeterActive;
MemSequence mseqPerimeterAvtive;

// ************************************
// Goto Area Behaviour
//*************************************
TsetMowBehaviour setMowBehaviour;
TGotoAreaX gotoAreaX;
TARrotate90CC arRotate90CC;
MemSequence mseqGotoAreaX;
Selector selGotoAreaX;


// ************************************
// Charging Behaviour
//*************************************

Selector selCharging;
TchargeRelayOn chargeRelayOn;


// ************************************
// Find Perimeter Behaviour
//*************************************

Selector selFindPerimeter;

// ************************************
// Perimeter Tracking Behaviour
//*************************************

TMotorStopFast MotorStopFast;
MemSequence mseqDockingStation;
TperTrackChargingStationReached perTrackChargingStationReached;


//TFLRotateCC FLRotateCC;
//TFLRotateCW FLRotateCW;
TtrackPerimeter trackPerimeter;


Failer    FLFailer;
//MemSequence mseqOvershootLeft;
//MemSequence mseqOvershootRight;
Selector selPerTracking;
//MemSelector mselFollowLine;


// ************************************
// Escape Obstacle Inside Perimeter
//*************************************

MemSequence mseqEscBackCoilOutside;
MemSequence mseqEscRotate;

TEscRotateCW escRotateCW;
MemSequence mseqEscRotateCW;


TEscRotateCC escRotateCC;
MemSequence mseqEscRotateCC;


TForward20 forward20;
MemSequence mseqEscForward;


TReverse180  reverse180;
MemSequence mseqEscBackward;

MemSelector mselEscabeObstacle;

Selector selEscabeObstacle1;

TdnBumperActivated dnBumperActivated;
// ************************************
// Perimeter Outside Behaviour
//*************************************



TRotate308 rotate308;
TRotateWorkx rotateWorkx;
MemSelector mselRotate;


//TdnPerOutsideActivated dnPerOutsideActivated;


// ************************************
// BatLow Behaviour
//*************************************

TCruiseBatLow CruiseBatLow;
Sequence seqMowBatLow;

// ************************************
// Check2 Behaviour
//*************************************

Selector selCheck2;


TCheck2LeftCoilSignal Check2LeftCoilSignal;
TCheck2RightCoilSignal Check2RightCoilSignal;
TCheck2BackCoilSignal Check2BackCoilSignal;

TCheck2BackCoilSignalAreaX Check2BackCoilSignalAreaX;

TCheck2PerSignal Check2PerSignal;
TCheck2AllCoilsOutside Check2AllCoilsOutside;

// ************************************
// Cruise Behaviour
//*************************************

TCruiseSpiral cruiseSpiral;

TCruiseStartMowMotor CruiseStartMowMotor;

TCruiseRotCW CruiseRotCW;
TCruiseStopped CruiseStopped;
TCruiseToPerimeter CruiseToPerimeter;



MemSequence mseqFindPerimeter;

TCruiseSpeedToMotor CruiseSpeedToMotor;
TCruiseObstacleNear CruiseObstacleNear;
TCruisePerimeterNear CruisePerimeterNear;
TCruiseHighSpeed CruiseHighSpeed;
Selector selCruiseSpeed;
Selector selCruise;
TdnSetbbShortWayCounter dnSetbbShortWayCounter;


Selector selPerimeterTracking;
Selector selMowing;

void TBehaviour::reset()
{
    // Init black board variables
    bb.lastNodeLastRun = NULL;
    bb.lastNodeCurrentRun = NULL;

    bb.cruiseSpeed = 0;
    bb.timeCruiseSpeedSet = 0;
    bb.shortWayCounter = 0;

    bb.flagBumperInsidePerActivated  = false;
    //bb.flagPerimeterActivated  = false;
    bb.flagBumperOutsidePerActivated = false;
    bb.flagCruiseSpiral = false;
    
    errorHandler.setInfo(F("bht->reset diable flagCruiseSpiral\r\n"));

    bb.flagCoilFirstOutside = CO_BOTH;
    bb.flagCoilOutsideAfterOverrun  = CO_BOTH;
    bb.flagForceSmallRotAngle=0;

    bb.flagEscabeObstacleConFlag = FEO_NONE;

    bb.flagForceRotateDirection = FRD_NONE;

    bb.flagMowMode = MOWMODE_308;
    bb.arcRotateXArc = 0;

    bb.lastTimeSpiralStarted = millis() + 60000ul; // Erst eine Minute nach Reset warten, bevor Sprirale aktiviert werden kann.
    
    bb.areaTargetDistanceInMeter = 2;

    bb.flagGotoAreaXFirstCall = false;
    bb.flagGoHome = false;

    if(bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
        bb.driveDirection = DD_REVERSE_INSIDE;
    } else {
        bb.driveDirection = DD_FORWARD;
    }

    bb.setBehaviour(BH_NONE);

    behaviorTree.reset(bb);
}

void TBehaviour::setup()
{
    reset();


// ************************************
// Conditions Behaviour
//*************************************

    conBackCoilOutAndDDReverse.nodeName = (char*)"conBackCoilOutAndDDReverse";

    conFEO_BACK180.nodeName = (char*)"conFEO_BACK180";
    conFEO_BACK180_BCO.nodeName = (char*)"conFEO_BACK180_BCO";
    conFEO_FWD20.nodeName = (char*)"conFEO_FWD20";
    conFEO_ROTCC.nodeName = (char*)"conFEO_ROTCC";
    conFEO_ROTCW.nodeName = (char*)"conFEO_ROTCW";
    conFEO_ROT.nodeName = (char*)"conFEO_ROT";
    conFEO_BACKINSIDE.nodeName = (char*)"conFEO_BACKINSIDE";


    conWasDirectionForward.nodeName = (char*)"conWasDirectionForward";
    conWasDirectionForward20.nodeName = (char*)"conWasDirectionForward20";
    conWasDirectionForwardInside.nodeName = (char*)"conWasDirectionForwardInside";
    conWasDirectionOverrun.nodeName = (char*)"conWasDirectionOverrun";
    conWasDirectionReverseObstacle.nodeName = (char*)"conWasDirectionReverseObstacle";
    conWasDirectionReverseInside.nodeName = (char*)"conWasDirectionReverseInside";
    conWasDirectionRotateCC.nodeName = (char*)"conWasDirectionRotateCC";
    conWasDirectionRotateCW.nodeName = (char*)"conWasDirectionRotateCW";
    //conNOTflagPerimeterActivated.nodeName = "conNOTflagPerimeterActivated";

    conBumperActive.nodeName = (char*)"conBumperActive";
    conPerOutside.nodeName = (char*)"conPerOutside"; //Condition

    //conStopOvershootLeft.nodeName = "conStopOvershootLeft";
    //conStopOvershootRight.nodeName = "conStopOvershootRight";
    conBatLow.nodeName= (char*)"conBatLow";

    conRightCoilOutside.nodeName= (char*)"conRightCoilOutside";
    conLeftCoilOutside.nodeName= (char*)"conLeftCoilOutside";

    conAreaReached.nodeName= (char*)"conAreaReached";

// ************************************
// Decorator Nodes Behaviour
//*************************************

    dnCruiseSpiral.nodeName= (char*)"dnCruiseSpiral";
    dnCruiseSpiral.setChild(&cruiseSpiral);
    
    dnMowModeWorkx.nodeName= (char*)"dnMowModeWorkx";
    dnMowMode308.nodeName= (char*)"dnMowMode308";
    
    dnMowModeWorkx.setChild(&rotateWorkx);
    dnMowMode308.setChild(&rotate308);

    dnBumpPeriActivated.nodeName= (char*)"dnBumpPeriActivated";

    dnBumpPeriActivated.setChild(&mselEscObstacleOutside);


    dnMowing.nodeName= (char*)"dnMowing";
    dnMowing.setChild(&selMowing);

    dnPermeterTracking.nodeName= (char*)"dnPermeterTracking";
    dnPermeterTracking.setChild(&selPerimeterTracking);

    dnCharging.nodeName= (char*)"dnCharging";
    dnCharging.setChild(&selCharging);

    dnGotoAreaX.nodeName= (char*)"dnGotoAreaX";
    dnGotoAreaX.setChild(&selGotoAreaX);

    dnFindPerimeter.nodeName= (char*)"dnFindPerimeter";
    dnFindPerimeter.setChild(&selFindPerimeter);

    dnWaitDockingStation.nodeName= (char*)"dnWaitDockingStation";
    dnWaitDockingStation.setChild(&conInDockingStation);
    dnWaitDockingStation.setWaitMillis(2000);



    dnWaitGotoArea.nodeName= (char*)"dnWaitGotoArea";
    dnWaitGotoArea.setChild(&gotoAreaX);
    dnWaitGotoArea.setWaitMillis(2000);


// ************************************
// Escape Obstacle Outside Perimeter
//*************************************


    mseqEscOOPReverse.nodeName = (char*)"mseqEscOOPReverse";
    mselEscObstacleOutside.nodeName = (char*)"mselEscObstacleOutside";
    setDD_FORWARD.nodeName = (char*)"setDD_FORWARD";
    seqEscOOPRot.nodeName = (char*)"seqEscOOPRot";
    seqEscOOPRotCC.nodeName = (char*)"seqEscOOPRotCC";
    seqEscOOPRotCW.nodeName = (char*)"seqEscOOPRotCW";

    seqEscOOPRot.addChildren(&mseqEscRotate,&setDD_FORWARD);
    seqEscOOPRotCC.addChildren(&mseqEscRotateCC,&setDD_FORWARD);
    seqEscOOPRotCW.addChildren(&mseqEscRotateCW,&setDD_FORWARD);

    mseqEscOOPReverse.addChildren(&conFEO_BACKINSIDE,&mseqReverseInside,&reverseFurtherInside,&mselRotate);
    mselEscObstacleOutside.addChildren(&mseqEscOOPReverse,&seqEscOOPRotCW ,&seqEscOOPRotCC, &seqEscOOPRot);


// ************************************
// Bumper Behaviour
//*************************************

    selEscapeAlgorithm.nodeName = (char*)"selEscapeAlgorithm";
    hardstop.nodeName = (char*)"hardstop";

    freeBumper.nodeName = (char*)"freeBumper";
    mseqBumperActive.nodeName = (char*)"mseqBumperActive";

    mseqBumperActive.addChildren(&conBumperActive,&hardstop,&freeBumper,&selEscapeAlgorithm );


// ************************************
// Free Perimeter Behaviour
//*************************************

    mseqPerBackCoilOutside.nodeName = (char*)"mseqPerBackCoilOutside";

    setFlagForceSmallRotAngle.nodeName = (char*)"flagForceSmallRotAngle";
    mseqPerimeterForward20.nodeName = (char*)"mseqPerimeterForward20";

    mseqPerimeterForwardInside.nodeName = (char*)"mseqPerimeterForwardInside";

    setArc90CW.nodeName = (char*)"setArc90CW";
    setArc90CC.nodeName = (char*)"setArc90CC";
    rotateX.nodeName = (char*)"rotateX";
    mseqPRRCO.nodeName = (char*)"mseqPRRCO";
    mseqPRLCO.nodeName = (char*)"mseqPRLCO";
    mselPerimeterReverse.nodeName = (char*)"mseqPRLCO";


    motorStop.nodeName = (char*)"motorStop";
    perRotateInsideCW.nodeName = (char*)"perRotateInsideCW";
    perRotateInsideCC.nodeName = (char*)"perRotateInsideCC";
    mseqPerimeterRotCC.nodeName = (char*)"mseqPerimeterRotCC";
    mseqPerimeterRotCW.nodeName = (char*)"mseqPerimeterRotCW";


    overRun.nodeName = (char*)"overRun";
    rotateBackCoilInside2.nodeName = (char*)"rotateBackCoilInside2";
    rotateBackCoilInside.nodeName = (char*)"rotateBackCoilInside";
    mseqReverseInside.nodeName = (char*)"mseqReverseInside";

    reverseInside.nodeName = (char*)"reverseInside";
    reverseFurtherInside.nodeName = (char*)"reverseFurtherInside";


    mseqPerimeterRevInside.nodeName = (char*)"mseqPerimeterRevInside";
    mseqPerimeterOverrun.nodeName = (char*)"seqPerimeterOverrun";

    mseqPerimeterForward.nodeName = (char*)"mseqPerimeterForward";
    mselPerimeterActive.nodeName = (char*)"mselPerimeterActive";
    mseqPerimeterAvtive.nodeName = (char*)"mseqPerimeterAvtive";

    forwardInside.nodeName = (char*)"forwardInside";
    mseqPerimeterReverse.nodeName = (char*)"mseqPerimeterReverse";


    mseqPerBackCoilOutside.addChildren(&conBackCoilOutAndDDReverse,&motorStop,&rotateBackCoilInside2,&setDD_FORWARD);

    mseqPerimeterForward20.addChildren(&conWasDirectionForward20, &setFlagForceSmallRotAngle, &motorStop,&seqEscOOPRot);

    mseqPerimeterRevInside.addChildren(&conWasDirectionReverseInside,&motorStop,&mseqReverseInside,&reverseFurtherInside, &mselRotate);


    mseqReverseInside.addChildren(&rotateBackCoilInside,&reverseInside);


    mseqPerimeterOverrun.addChildren(&conWasDirectionOverrun,&motorStop,&mseqReverseInside, &reverseFurtherInside,&mselRotate);

    mseqPerimeterForwardInside.addChildren(&conWasDirectionForwardInside,&motorStop,&forwardInside);

    //Reverse
    mseqPRRCO.addChildren(&conRightCoilOutside,&motorStop, &setArc90CW, &rotateX, &mseqReverseInside, &reverseFurtherInside, &mselRotate);
    mseqPRLCO.addChildren(&conLeftCoilOutside,&motorStop, &setArc90CC, &rotateX, &mseqReverseInside, &reverseFurtherInside, &mselRotate);
    mselPerimeterReverse.addChildren(&mseqPRLCO,&mseqPRRCO, &forwardInside);

    mseqPerimeterReverse.addChildren(&conWasDirectionReverseObstacle,&motorStop,&mselPerimeterReverse);

    //mseqPerimeterRotCC.addChildren(&conWasDirectionRotateCC,&motorStop,&perRotateInsideCW, &mselRotate, &setDD_FORWARD);
    //mseqPerimeterRotCW.addChildren(&conWasDirectionRotateCW,&motorStop,&perRotateInsideCC, &mselRotate, &setDD_FORWARD);

    mseqPerimeterRotCC.addChildren(&conWasDirectionRotateCC,&motorStop,&mseqReverseInside,&reverseFurtherInside, &mselRotate);
    mseqPerimeterRotCW.addChildren(&conWasDirectionRotateCW,&motorStop,&mseqReverseInside,&reverseFurtherInside, &mselRotate);

    mseqPerimeterForward.addChildren(&conWasDirectionForward,&overRun,&mseqReverseInside,&reverseFurtherInside, &mselRotate);
    mselPerimeterActive.addChildren(&mseqPerimeterForward20, &mseqPerimeterForward,&mseqPerimeterOverrun,&mseqPerimeterRevInside, &mseqPerimeterRotCC,&mseqPerimeterRotCW,&mseqPerimeterReverse,&mseqPerimeterForwardInside);
    mseqPerimeterAvtive.addChildren(&conPerOutside,&mselPerimeterActive);


// ************************************
// GoTo Area Behaviour
//*************************************
    selGotoAreaX.nodeName = (char*)"selGotoAreaX";
    mseqGotoAreaX.nodeName = (char*)"mseqGotoAreaX";
    gotoAreaX.nodeName = (char*)"gotoAreaX";
    arRotate90CC.nodeName = (char*)"arRotate90CC";
    setMowBehaviour.nodeName = (char*)"setMowBehaviour";

    mseqGotoAreaX.addChildren(&conAreaReached,&motorStop,&arRotate90CC,&mseqReverseInside,&reverseFurtherInside,&setMowBehaviour);
    selGotoAreaX.addChildren(&CruiseStartMowMotor,&mseqBumperActive,&mseqGotoAreaX,&dnWaitGotoArea);


// ************************************
// Charging Behaviour
//*************************************

    selCharging.nodeName = (char*)"selCharging";
    chargeRelayOn.nodeName = (char*)"chargeRelayOn";
    selCharging.addChildren(&chargeRelayOn);



// ************************************
// Find Perimeter Behaviour
//*************************************

    selFindPerimeter.nodeName = (char*)"selCharging";
    mseqFindPerimeter.nodeName= (char*)"mseqFindPerimeter";

    mseqFindPerimeter.addChildren(&CruiseToPerimeter,&CruiseStopped,&CruiseRotCW,&CruiseStopped);
    selFindPerimeter.addChildren(&mseqBumperActive, &dnBumpPeriActivated, &dnBumperActivated, &selCruiseSpeed, &mseqFindPerimeter);

// ************************************
// Perimeter Tracking
//*************************************

    MotorStopFast.nodeName = (char*)"motorStopFast";
    mseqDockingStation.nodeName = (char*)"mseqDockingStation";
    perTrackChargingStationReached.nodeName = (char*)"perTrackChargingStationReached";


    //FLRotateCC.nodeName = "fLRotateCC";
    //FLRotateCW.nodeName = "fLRotateCW";
    trackPerimeter.nodeName = (char*)"trackPerimeter";


    FLFailer.nodeName = (char*)"fLFailer";
    //mseqOvershootLeft.nodeName = "mseqOvershootLeft";
    //mseqOvershootRight.nodeName = "mseqOvershootRight";
    selPerTracking.nodeName = (char*)"selPerTracking";
    //mselFollowLine.nodeName = "mselFollowLine";


    mseqDockingStation.addChildren(&conInDockingStation,&MotorStopFast,&dnWaitDockingStation, &perTrackChargingStationReached);

    FLFailer.setChild(&motorStop);
    //mseqOvershootLeft.addChildren(&conStopOvershootLeft,&motorStop,&FLRotateCW,&FLFailer );
    //mseqOvershootRight.addChildren(&conStopOvershootRight,&motorStop,&FLRotateCC,&FLFailer );
    //mselFollowLine.addChildren(&mseqOvershootLeft,&mseqOvershootRight);
    //selFollowLine.addChildren(&mselFollowLine,&trackPerimeter );
    selPerTracking.addChildren(&trackPerimeter );

    selPerimeterTracking.nodeName = (char*)"selPerimeterTracking";
    selPerimeterTracking.addChildren(&mseqBumperActive,&mseqDockingStation,&selPerTracking);


// ************************************
// Escape Obstacle Inside Perimeter
//*************************************

    mseqEscBackCoilOutside.nodeName = (char*)"mseqEscBackCoilOutside";

    mseqEscRotate.nodeName = (char*)"mseqEscRotate";

    escRotateCC.nodeName = (char*)"escRotateCC";
    mseqEscRotateCW.nodeName = (char*)"mseqEscRotateCW";


    escRotateCW.nodeName = (char*)"escRotateCW";
    mseqEscRotateCC.nodeName = (char*)"mseqEscRotateCC";



    forward20.nodeName = (char*)"forward20";
    mseqEscForward.nodeName = (char*)"mseqEscForward";

    reverse180.nodeName = (char*)"reverse180";
    mseqEscBackward.nodeName = (char*)"mseqEscBackward";


    mselEscabeObstacle.nodeName = (char*)"mselEscabeObstacle";

    selEscabeObstacle1.nodeName = (char*)"selEscabeObstacle1";
    dnBumperActivated.nodeName = (char*)"dnBumperActivated";




    mseqEscRotate.addChildren(&conFEO_ROT,&mselRotate);

    mseqEscRotateCW.addChildren(&conFEO_ROTCW,&escRotateCW);
    mseqEscRotateCC.addChildren(&conFEO_ROTCC,&escRotateCC);


    mseqEscForward.addChildren(&conFEO_FWD20,&forward20, &mselRotate);
    mseqEscBackward.addChildren(&conFEO_BACK180,&reverse180, &mselRotate);
    mselEscabeObstacle.addChildren(&mseqEscBackward, &mseqEscForward, &mseqEscRotateCC ,&mseqEscRotateCW,&mseqEscRotate, &mseqPerimeterForwardInside) ;

    mseqEscBackCoilOutside.addChildren(&conFEO_BACK180_BCO,&motorStop, &mselRotate);
    selEscabeObstacle1.addChildren(&mseqEscBackCoilOutside,&mselEscabeObstacle);
    dnBumperActivated.setChild(&selEscabeObstacle1);

    // ************************************
    // Perimeter Outside Behaviour
    //*************************************

    rotate308.nodeName = (char*)"rotate308";
    rotateWorkx.nodeName = (char*)"rotateWorkx";
    mselRotate.nodeName = (char*)"mselRotate";

    //dnPerOutsideActivated.nodeName="dnPerOutsideActivated";

    mselRotate.addChildren(&dnMowModeWorkx,&dnMowMode308);
    //dnPerOutsideActivated.setChild(&mselRotate);


    // ************************************
    // BatLow Behaviour
    //*************************************

    CruiseBatLow.nodeName= (char*)"cruiseBatLow";
    seqMowBatLow.nodeName= (char*)"seqMowBatLow";
    seqMowBatLow.addChildren( &conBatLow, &CruiseBatLow);


    // ************************************
    // Check2 Behaviour
    //*************************************

    selCheck2.nodeName = (char*)"selCheck2";

    Check2LeftCoilSignal.nodeName = (char*)"Check2LeftCoilSignal";
    Check2RightCoilSignal.nodeName = (char*)"Check2RightCoilSignal";
    Check2BackCoilSignal.nodeName = (char*)"Check2BackCoilSignal";


    Check2BackCoilSignalAreaX.nodeName= (char*)"Check2BackCoilSignalAreaX";

    Check2PerSignal.nodeName = (char*)"check2PerSignal";
    Check2AllCoilsOutside.nodeName = (char*)"check2AllCoilsOutside";
    selCheck2.addChildren( &Check2PerSignal,&Check2AllCoilsOutside,&Check2BackCoilSignalAreaX,&Check2LeftCoilSignal,&Check2RightCoilSignal,&Check2BackCoilSignal);

    // ************************************
    // Cruise Behaviour
    //*************************************

    cruiseSpiral.nodeName = (char*)"cruiseSpiral";
    
    CruiseStartMowMotor.nodeName = (char*)"cruiseStartMowMotor";
    CruiseRotCW.nodeName= (char*)"cruiseRotCW";
    CruiseStopped.nodeName= (char*)"cruiseStopped";
    CruiseToPerimeter.nodeName= (char*)"cruiseToPerimeter";

    CruiseObstacleNear.nodeName= (char*)"cruiseObstacleNear";
    CruisePerimeterNear.nodeName= (char*)"cruisePerimeterNear";
    CruiseHighSpeed.nodeName= (char*)"cruiseHighSpeed";
    CruiseSpeedToMotor.nodeName= (char*)"cruiseSpeedToMotor";
    selCruiseSpeed.nodeName= (char*)"selCruiseSpeed";
    selCruise.nodeName= (char*)"selCruise";
    dnSetbbShortWayCounter.nodeName= (char*)"dnSetbbShortWayCounter";

    selCruiseSpeed.addChildren( &CruiseObstacleNear, &CruisePerimeterNear, &CruiseHighSpeed);
    selCruise.addChildren(&selCruiseSpeed,&CruiseSpeedToMotor);
    dnSetbbShortWayCounter.setChild(&selCruise);

    // ************************************
    // Mowing
    //*************************************

    selMowing.nodeName = (char*)"selMowing";

    selMowing.addChildren(&CruiseStartMowMotor, &mseqBumperActive, &dnBumpPeriActivated, &mseqPerBackCoilOutside, &mseqPerimeterAvtive,  &dnBumperActivated, &seqMowBatLow, &dnCruiseSpiral, &dnSetbbShortWayCounter);




    // ************************************
    // Root
    //*************************************

    selRoot.nodeName = (char*)"rootSel";
    selRoot.addChildren(&dnCharging,&selCheck2,&dnGotoAreaX,&dnPermeterTracking,&dnFindPerimeter,&dnMowing);

    behaviorTree.setRootNode (&selRoot);
}

void TBehaviour::loop()
{
    behaviorTree.tick(bb);

}

