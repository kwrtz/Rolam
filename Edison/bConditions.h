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
#ifndef BH_CONDITIONS_H
#define BH_CONDITIONS_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"


class TConBatLow: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TConBatLow() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside() && bb.perimeterSensoren.isBackInside() && bb.driveDirection == DD_FORWARD) {
            if (bb.batterieSensor.isVoltageLow() || bb.flagGoHome ) {
                sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
                errorHandler.setInfo();
                return BH_SUCCESS;
            }
        }
        return BH_FAILURE;
    }
};


class TConBackCoilOutAndDDReverse: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TConBackCoilOutAndDDReverse() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if( bb.perimeterSensoren.isBackOutside() && 
           (bb.driveDirection == DD_REVERSE_INSIDE) && 
           (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) 
           ) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }
        return BH_FAILURE;
    }
};

class TConFEO_BACK180: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConFEO_BACK180() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEscabeObstacleConFlag == FEO_BACK180) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};

class TConFEO_BACK180_BCO: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConFEO_BACK180_BCO() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if ( (bb.flagEscabeObstacleConFlag == FEO_BACK180)  && bb.perimeterSensoren.isBackOutside()  && (bb.driveDirection == DD_REVERSE_ESC_OBST)) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConFEO_FWD20: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConFEO_FWD20() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEscabeObstacleConFlag == FEO_FWD20) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConFEO_ROTCC: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConFEO_ROTCC() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEscabeObstacleConFlag == FEO_ROTCC) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConFEO_ROTCW: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConFEO_ROTCW() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEscabeObstacleConFlag == FEO_ROTCW) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConFEO_ROT: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConFEO_ROT() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEscabeObstacleConFlag == FEO_ROT) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConFEO_BACKINSIDE: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConFEO_BACKINSIDE() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEscabeObstacleConFlag == FEO_BACKINSIDE) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};

class TConWasDirectionForward: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConWasDirectionForward() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.driveDirection == DD_FORWARD || bb.driveDirection==DD_SPIRAL_CW ) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConWasDirectionForward20: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConWasDirectionForward20() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.driveDirection == DD_FORWARD20) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};

class TConWasDirectionForwardInside: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConWasDirectionForwardInside() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.driveDirection == DD_FORWARD_INSIDE) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};

class TConWasDirectionReverseObstacle: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConWasDirectionReverseObstacle() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.driveDirection == DD_REVERSE_ESC_OBST) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConWasDirectionOverrun: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConWasDirectionOverrun() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.driveDirection == DD_OVERRUN) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};




class TConWasDirectionReverseInside: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConWasDirectionReverseInside() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.driveDirection == DD_REVERSE_INSIDE) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};

class TConWasDirectionRotateCW: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConWasDirectionRotateCW() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.driveDirection == DD_ROTATECW) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConWasDirectionRotateCC: public Node    // Each task will be a class (derived from Node of course).
{
public:

    TConWasDirectionRotateCC() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.driveDirection == DD_ROTATECC) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};



class TConPerOutside: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TConPerOutside() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
            bb.flagCoilFirstOutside = CO_BOTH;
            sprintf(errorHandler.msg,"!03,->%s flagCoilFirstOutside = CO_BOTH\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        } else if (bb.perimeterSensoren.isLeftOutside()) {
            bb.flagCoilFirstOutside = CO_LEFT;
            sprintf(errorHandler.msg,"!03,->%s flagCoilFirstOutside = CO_LEFT\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        } else if (bb.perimeterSensoren.isRightOutside()) {
            bb.flagCoilFirstOutside= CO_RIGHT;
            sprintf(errorHandler.msg,"!03,->%s flagCoilFirstOutside= CO_RIGHT\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};


class TConBumperActive: public Node    // Condition
{
private:

public:

    TConBumperActive() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.bumperSensor.isBumperActivated()) {   //for bumper.  hard stop is activated by bumper sensor. Therfore I have to check this first before doing further steps
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }

        return BH_FAILURE;
    }
};





class TConLeftCoilOutside: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TConLeftCoilOutside() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside()) {
            sprintf(errorHandler.msg,"!03,->%s BH_SUCCESS\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }
        sprintf(errorHandler.msg,"!03,->%s BH_FAILURE\r\n",nodeName);
        errorHandler.setInfo();
        return BH_FAILURE;
    }
};

class TConRightCoilOutside: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TConRightCoilOutside() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightOutside()) {
            sprintf(errorHandler.msg,"!03,->%s BH_SUCCESS\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }
        sprintf(errorHandler.msg,"!03,->%s BH_FAILURE\r\n",nodeName);
        errorHandler.setInfo();
        return BH_FAILURE;
    }
};



class TConInDockingStation: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TConInDockingStation() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.chargeSystem.isInChargingStation()) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }
        //errorHandler.setInfo("!03,TchargeSystem NOT detected\r\n");
        return BH_FAILURE;
    }
};


class TconAreaReached: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TconAreaReached() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.motor.getDistanceInMeterAreax() >= bb.areaTargetDistanceInMeter ) {
            sprintf(errorHandler.msg,"!03,->%s\r\n",nodeName);
            errorHandler.setInfo();
            return BH_SUCCESS;
        }
        //errorHandler.setInfo("!03,TchargeSystem NOT detected\r\n");
        return BH_FAILURE;
    }
};

#endif

