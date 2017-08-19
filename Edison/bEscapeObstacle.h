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

#ifndef BH_ESCABEOBSTACLE_H
#define BH_ESCABEOBSTACLE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"




class TReverse180: public Node    // Each task will be a class (derived from Node of course).
{
private:
    unsigned long lastTimeCalled;
    int count;
public:

    TReverse180(): lastTimeCalled(0),count(0)  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;

        unsigned long now = millis();
        if( now-lastTimeCalled < 15000) {
            count ++;
            if (count >2) {
                count = 0;
            }
        } else {
            count = 0;
        }

        switch(count) {
            case 1  :
                bb.motor.rotateAngle(-225,bb.cruiseSpeed); // -45 gad zurueckfahren
                break; //optional
            case 2  :
                bb.motor.rotateAngle(-360,bb.cruiseSpeed); // -180 gad zurueckfahren
                break; //optional

                // you can have any number of case statements.
            default : //Optional
                bb.motor.rotateAngle(-135,bb.cruiseSpeed); // -90 grad zurueckfahren
        }


        bb.driveDirection = DD_REVERSE_ESC_OBST;
        lastTimeCalled = millis();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,Reverse180 too long in state\r\n");
        }

        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }

};


class TForward20: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TForward20()  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
        bb.motor.rotateAngle(5,bb.cruiseSpeed); // 20 grad vorwÃ¤rtsfahren
        bb.driveDirection = DD_FORWARD;
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,TForward20 too long in state\r\n");
        }

        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }


};




class TEscRotateCC: public Node    // Each task will be a class (derived from Node of course).
{
private:
    bool flagDoRotation;
    unsigned long lastTimeCalled;
    int arcEscRotate;
public:

    TEscRotateCC (): flagDoRotation(false),lastTimeCalled(0),arcEscRotate(0) {}

    virtual void onInitialize(Blackboard& bb) {
        unsigned long now = millis();
        bb.flagForceRotateDirection = FRD_CC;
        if( now-lastTimeCalled < 15000) {
            arcEscRotate += 50;

            if(bb.flagForceSmallRotAngle > 0) {
                arcEscRotate = myRandom(50, 60);
                bb.flagForceSmallRotAngle--;
            }

            bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
            bb.driveDirection =  DD_ROTATECC;
            bb.motor.turnTo(-1*arcEscRotate,bb.cruiseSpeed);
            flagDoRotation = true;
            sprintf(errorHandler.msg,"!03,CC flagDoRotation = true deltatime: %lu;\r\n",now-lastTimeCalled);
            errorHandler.setInfo();
        } else {
            flagDoRotation = false;
            arcEscRotate = 0;
            errorHandler.setInfo("!03,CC flagDoRotation = false;\r\n");
        }

        lastTimeCalled = millis();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,EscRotateCC  too long in state\r\n");
        }

        // Wenn erste mal innerhalbe 15sek aufgerufen, dann beenden um einfach wieder geradeaus zu fahren
        if (flagDoRotation == false ) {
            return BH_SUCCESS;
        }


        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }
        return BH_RUNNING;
    }
};


class TEscRotateCW: public Node    // Each task will be a class (derived from Node of course).
{
private:
    bool flagDoRotation;
    unsigned long lastTimeCalled;
    int arcEscRotate;
public:

    TEscRotateCW (): flagDoRotation(false),lastTimeCalled(0),arcEscRotate(0) {}

    virtual void onInitialize(Blackboard& bb) {
        unsigned long now = millis();
        bb.flagForceRotateDirection = FRD_CW;
        if( now-lastTimeCalled < 15000) {
            arcEscRotate += 50;

            if(bb.flagForceSmallRotAngle > 0) {
                arcEscRotate = myRandom(50, 60);
                bb.flagForceSmallRotAngle--;
            }

            bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
            bb.driveDirection =  DD_ROTATECW;
            bb.motor.turnTo(1*arcEscRotate,bb.cruiseSpeed);
            flagDoRotation = true;
            sprintf(errorHandler.msg,"!03,CW flagDoRotation = true deltatime: %lu;\r\n",now-lastTimeCalled);
            errorHandler.setInfo();
        } else {
            flagDoRotation = false;
            arcEscRotate = 0;
            errorHandler.setInfo("!03,CW flagDoRotation = false;\r\n");
        }
        lastTimeCalled = millis();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,EscRotateCW  too long in state\r\n");
        }


        // Wenn erste mal innerhalbe 15sek aufgerufen, dann beenden um einfach wieder geradeaus zu fahren
        if (flagDoRotation == false ) {
            return BH_SUCCESS;
        }


        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }
        return BH_RUNNING;
    }
};

#endif

