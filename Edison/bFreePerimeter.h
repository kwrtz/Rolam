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

#ifndef BH_PERIMETER_H
#define BH_PERIMETER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"



class TOverRun: public Node    // Each task will be a class (derived from Node of course).
{
private:
    long weg;
public:

    TOverRun()  {}

    virtual void onInitialize(Blackboard& bb) {

        weg = mapl(bb.cruiseSpeed, bb.CRUISE_SPEED_OBSTACLE, bb.CRUISE_SPEED_HIGH, 140, 0);//50//90 //140



        bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
        bb.motor.rotateAngle(weg, bb.cruiseSpeed);
        bb.driveDirection = DD_OVERRUN;
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,Runover too long in node\r\n");
        }

        if (bb.motor.isPositionReached()) {
         /*   
            if ( bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
                //bb.driveDirection =  DD_FORWARD;
                //return BH_FAILURE;
                bb.flagCoilOutsideAfterOverrun  = CO_BOTH;

            } else if ( bb.perimeterSensoren.isLeftOutside() ) {
                bb.flagCoilOutsideAfterOverrun  = CO_LEFT;
            } else if ( bb.perimeterSensoren.isRightOutside() ) {
                bb.flagCoilOutsideAfterOverrun  = CO_RIGHT;
            }
          */
            return BH_SUCCESS;
        }



        return BH_RUNNING;
    }

};


class TRotateBackCoilInside2: public Node    // Each task will be a class (derived from Node of course).
{
private:
    bool turnActivated;
public:

    TRotateBackCoilInside2():turnActivated(false)  {}

    virtual void onInitialize(Blackboard& bb) {

        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;

        if(bb.perimeterSensoren.magnetudeB < 0) {
            if(bb.flagCoilFirstOutside == CO_BOTH ) {
                // Do nothing
                turnActivated = true;
                bb.motor.turnTo(-90,bb.cruiseSpeed);
                turnActivated=true;
                bb.driveDirection =  DD_ROTATECC;
                bb.flagForceRotateDirection = FRD_CC;
                bb.flagForceSmallRotAngle=1;
                sprintf(errorHandler.msg,"!03,%s CC-90\r\n",nodeName);
                errorHandler.setInfo();

            } else if(bb.flagCoilFirstOutside == CO_LEFT) {
                bb.motor.turnTo(-30,bb.cruiseSpeed);
                turnActivated=true;
                bb.driveDirection =  DD_ROTATECC;
                bb.flagForceRotateDirection = FRD_CC;
                bb.flagForceSmallRotAngle=1;
                sprintf(errorHandler.msg,"!03,%s CC\r\n",nodeName);
                errorHandler.setInfo();

            } else if(bb.flagCoilFirstOutside == CO_RIGHT) {
                bb.motor.turnTo(30,bb.cruiseSpeed);
                turnActivated=true;
                bb.driveDirection =  DD_ROTATECW;
                bb.flagForceRotateDirection = FRD_CW;
                bb.flagForceSmallRotAngle=1;
                sprintf(errorHandler.msg,"!03,%s CW\r\n",nodeName);
                errorHandler.setInfo();
            } else {
                turnActivated = false;
            }
        } else {
            turnActivated = false;
        }
    }


    virtual NodeStatus onUpdate(Blackboard& bb) {

        // Wenn zeit in state > 10 Sek dann fehler oder anhalten undnochmal checken.
        if(getTimeInNode()> 10000) {
            sprintf(errorHandler.msg,"!03,->%s too long in state\r\n",nodeName);
            errorHandler.setInfo();
        }

        if(turnActivated) {
            if (bb.motor.isPositionReached()) {
                turnActivated = false;
                return BH_SUCCESS;
            } else {
                return BH_RUNNING;
            }
        }

        return BH_SUCCESS;
    }

};



class TRotateBackCoilInside: public Node    // Each task will be a class (derived from Node of course).
{
private:
    bool turnActivated;
public:

    TRotateBackCoilInside():turnActivated(false)  {}

    virtual void onInitialize(Blackboard& bb) {

        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;

        if(bb.perimeterSensoren.magnetudeB < 1000) {
            if(bb.flagCoilFirstOutside == CO_BOTH ) {
                // Do nothing
                turnActivated = true;
                bb.motor.turnTo(-90,bb.cruiseSpeed);
                turnActivated=true;
                bb.driveDirection =  DD_ROTATECC;
                bb.flagForceRotateDirection = FRD_CC;
                bb.flagForceSmallRotAngle=1;
                sprintf(errorHandler.msg,"!03,%s CC-90\r\n",nodeName);
                errorHandler.setInfo();

            } else if(bb.flagCoilFirstOutside == CO_LEFT) {
                bb.motor.turnTo(-30,bb.cruiseSpeed);
                turnActivated=true;
                bb.driveDirection =  DD_ROTATECC;
                bb.flagForceRotateDirection = FRD_CC;
                bb.flagForceSmallRotAngle=1;
                sprintf(errorHandler.msg,"!03,%s CC\r\n",nodeName);
                errorHandler.setInfo();

            } else if(bb.flagCoilFirstOutside == CO_RIGHT) {
                bb.motor.turnTo(30,bb.cruiseSpeed);
                turnActivated=true;
                bb.driveDirection =  DD_ROTATECW;
                bb.flagForceRotateDirection = FRD_CW;
                bb.flagForceSmallRotAngle=1;
                sprintf(errorHandler.msg,"!03,%s CW\r\n",nodeName);
                errorHandler.setInfo();
            }
        } else {
            turnActivated = false;
        }
    }


    virtual NodeStatus onUpdate(Blackboard& bb) {

        // Wenn zeit in state > 10 Sek dann fehler oder anhalten undnochmal checken.
        if(getTimeInNode()> 10000) {
            sprintf(errorHandler.msg,"!03,->%s too long in state\r\n",nodeName);
            errorHandler.setInfo();
        }

        if(turnActivated) {
            if (bb.motor.isPositionReached()) {
                turnActivated = false;
                return BH_SUCCESS;
            } else {
                return BH_RUNNING;
            }
        }

        return BH_SUCCESS;
    }

};

class TReverseInside: public Node    // Each task will be a class (derived from Node of course).
{

public:

    TReverseInside()  {}

    virtual void onInitialize(Blackboard& bb) {

        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;

        bb.motor.L->setSpeed(-bb.cruiseSpeed);
        bb.motor.R->setSpeed(-bb.cruiseSpeed);
        bb.driveDirection = DD_REVERSE_INSIDE;
        //sprintf(errorHandler.msg,"!03,#%s\r\n",nodeName);
        //errorHandler.setInfo();
        if ( bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
            bb.flagCoilOutsideAfterOverrun  = CO_BOTH;
            bb.motor.startDistanceMeasurementCoilOut(true);
        } else if ( bb.perimeterSensoren.isLeftOutside() ) {
            bb.flagCoilOutsideAfterOverrun  = CO_LEFT;
            bb.motor.startDistanceMeasurementCoilOut(false);
        } else if ( bb.perimeterSensoren.isRightOutside() ) {
            bb.flagCoilOutsideAfterOverrun  = CO_RIGHT;
            bb.motor.startDistanceMeasurementCoilOut(false);
        }
        
        

    }


    virtual NodeStatus onUpdate(Blackboard& bb) {

        // Wenn zeit in state > 10 Sek dann fehler oder anhalten undnochmal checken.
        if(getTimeInNode()> 10000) {
            sprintf(errorHandler.msg,"!03,->%s too long in state\r\n",nodeName);
            errorHandler.setInfo();
        }
        
         if (bb.perimeterSensoren.isLeftInside()){
             bb.motor.stopDistanceMeasurementLCoilOut();
         }    
         
         if (bb.perimeterSensoren.isRightInside()){
             bb.motor.stopDistanceMeasurementRCoilOut();
         }  

        if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
            return BH_SUCCESS;
        }

        if(getTimeInNode()> 3000 ) { // Fall nach 3 Sekunden noch nicht inside, dann Kurve einleiten

            if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightOutside()) {
                bb.motor.L->setSpeed(-(bb.cruiseSpeed-13));
                bb.motor.R->setSpeed(-(bb.cruiseSpeed+13));
                //sprintf(errorHandler.msg,"!03,->%s >3000\r\n",nodeName);
                //errorHandler.setInfo();
            }
            if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside()) {
                bb.motor.L->setSpeed(-(bb.cruiseSpeed+13));
                bb.motor.R->setSpeed(-(bb.cruiseSpeed-13));
                //sprintf(errorHandler.msg,"!03,->%s >3000\r\n",nodeName);
                //errorHandler.setInfo();
            }
        }



        return BH_RUNNING;
    }

};


class TReverseFurtherInside: public Node
{
private:
    long weg;
public:

    TReverseFurtherInside()  {}

    virtual void onInitialize(Blackboard& bb) {


        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
        bb.motor.rotateAngle(-40,bb.cruiseSpeed); // 40 grad zurueckfahren

    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if(getTimeInNode()> 5000) {
            errorHandler.setError("!03,reverseFurtherInside too long in state\r\n");
        }

        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        /*
                if(status != BH_ABORTED) {
                    bb.flagPerimeterActivated  = true;
                    sprintf(errorHandler.msg,"!03,*flagPerimeterActivated=true;\r\n");
                    errorHandler.setInfo();
                }
        */
    }

};


class TMotorStop: public Node
{
private:

public:

    TMotorStop() {}

    virtual void onInitialize(Blackboard& bb) {
        bb.motor.enableDefaultRamping();
        bb.motor.stop();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,motorStop too long in state\r\n");
        }

        if (bb.motor.isStopped()) {
            bb.cruiseSpeed = 0;
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }
};


class TPerRotateInsideCW: public Node
{
private:

public:

    TPerRotateInsideCW()  {}

    virtual void onInitialize(Blackboard& bb) {


        bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
        bb.motor.turnTo(380,bb.cruiseSpeed);
        bb.driveDirection =  DD_ROTATECW;
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 15000) {
            errorHandler.setError("!03,PerRotateInsideCW  too long in state\r\n");
        }


        if ( bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) { // wenn beide coils innen dann weitermähen
            bb.motor.pcL->reset(); // Mower fährt weiter in normalem Mähbetrieb. Daher Posisioncontrol deaktivieren. Robbi dreht weiter und wird durch cruise wieder gerade gelenkt.
            bb.motor.pcR->reset();
            bb.flagForceRotateDirection = FRD_CW;
            bb.flagForceSmallRotAngle = 1;
            return BH_SUCCESS;
        }

        if (bb.motor.isPositionReached()) {
            errorHandler.setError("!03,PerRotateInsideCWnot able to rotate both coils to inside\r\n");
        }

        return BH_RUNNING;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        /*
                if(status != BH_ABORTED) {
                    bb.flagPerimeterActivated  = true;
                    sprintf(errorHandler.msg,"!03,*flagPerimeterActivated=true;\r\n");
                    errorHandler.setInfo();
                }
        */
    }
};


class TPerRotateInsideCC: public Node
{
private:

public:

    TPerRotateInsideCC() {}

    virtual void onInitialize(Blackboard& bb) {

        bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

        bb.motor.turnTo(-380,bb.cruiseSpeed);
        bb.driveDirection =  DD_ROTATECC;

    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 15000) {
            errorHandler.setError("!03,PerRotateInsideCC  too long in state\r\n");
        }


        if ( bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) { // wenn beide coils innen dann weitermähen
            bb.motor.pcL->reset(); // Mower fährt weiter in normalem Mähbetrieb. Daher Posisioncontrol deaktivieren. Robbi dreht weiter und wird durch cruise wieder gerade gelenkt.
            bb.motor.pcR->reset();
            bb.flagForceRotateDirection = FRD_CC;
            bb.flagForceSmallRotAngle = 1;
            return BH_SUCCESS;
        }

        if (bb.motor.isPositionReached()) {
            errorHandler.setError("!03,PerRotateInsideCC not able to rotate both coils to inside\r\n");
        }

        return BH_RUNNING;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        /*
                if(status != BH_ABORTED) {
                    bb.flagPerimeterActivated  = true;
                    sprintf(errorHandler.msg,"!03,*flagPerimeterActivated=true;\r\n");
                    errorHandler.setInfo();
                }
        */
    }

};


/*
class TRotateOtherDirection: public Node    // Each task will be a class (derived from Node of course).
{
private:
    bool flagRotationFurther;
    unsigned long lastTimeCalled;
    int state;
    int arcRotate;
    int count;

public:

    TRotateOtherDirection():lastTimeCalled(0),state(0), arcRotate(0), count(0)  {}

    virtual void onInitialize(Blackboard& bb) {

        unsigned long now = millis();
        if( now-lastTimeCalled < 15000) {

            if(count > 2) {
                count = 0;
            } else {
                flagRotationFurther = true;
                arcRotate += 50;
                count++;
            }

        } else {
            flagRotationFurther = false;
            arcRotate = 0;
            count = 0;
        }

        bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

        if (bb.driveDirection ==  DD_ROTATECC) {
            bb.motor.turnTo(380,bb.cruiseSpeed);
            bb.driveDirection =  DD_ROTATECW;
        } else {
            bb.motor.turnTo(-380,bb.cruiseSpeed);
            bb.driveDirection =  DD_ROTATECC;
        }

        state = 0;
        lastTimeCalled = millis();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 15000) {
            bb.errorHandler.setError("!03,RotateOtherDirection  too long in state\r\n");
        }

        if(state == 0) {
            if ( bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside() && flagRotationFurther == false) { // wenn beide coils innen dann weitermähen
                bb.motor.pcL->reset(); // Mower fährt weiter in normalem Mähbetrieb. Daher Posisioncontrol deaktivieren. Robbi dreht weiter und wird durch cruise wieder gerade gelenkt.
                bb.motor.pcR->reset();
                return BH_SUCCESS;
            }

            if ( bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside() && flagRotationFurther == true) { // wenn beide coils innen aber innerhlab von 15sek erneut aufgerufen, dann nochmal arcRotate weiterdrehen
                if (bb.driveDirection ==  DD_ROTATECC) {
                    bb.motor.turnTo(-1* arcRotate,bb.cruiseSpeed);
                } else {
                    bb.motor.turnTo(arcRotate,bb.cruiseSpeed);
                }
                state = 1;
            }
        }

        if(state == 1) {
            if (bb.motor.isPositionReached()) {
                state = 0;
                return BH_SUCCESS;
            }
        }

        if(state == 0) {
            if (bb.motor.isPositionReached()) {
                bb.errorHandler.setError("!03,bMow STBH_ROTATE_OTHERDIRECTION not able to rotate both coils to inside\r\n");
            }
        }


        return BH_RUNNING;
    }


};
*/

class TForwardInside: public Node    // Each task will be a class (derived from Node of course).
{

public:

    TForwardInside()  {}

    virtual void onInitialize(Blackboard& bb) {

        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;

        // Wenn Winkel zum Perimeter zu flach, dann beim Rückwärtsfahren kurve fahren,
        if(bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
            bb.motor.L->setSpeed(bb.cruiseSpeed);
            bb.motor.R->setSpeed(bb.cruiseSpeed);
            bb.driveDirection = DD_FORWARD_INSIDE;
            //debug->printf("set bb.flagCoilsOutside= TD_BOTH)\r\n";
            sprintf(errorHandler.msg,"!03,TForwardInside\r\n");
            errorHandler.setInfo();
        } else if(bb.perimeterSensoren.isLeftOutside()) {
            bb.motor.L->setSpeed((bb.cruiseSpeed+13));
            bb.motor.R->setSpeed((bb.cruiseSpeed-13));
            bb.driveDirection = DD_FORWARD_INSIDE;
            sprintf(errorHandler.msg,"!03,TForwardInside\r\n");
            errorHandler.setInfo();
        } else if(bb.perimeterSensoren.isRightOutside()) {
            bb.motor.L->setSpeed((bb.cruiseSpeed-13));
            bb.motor.R->setSpeed((bb.cruiseSpeed+13));
            bb.driveDirection = DD_FORWARD_INSIDE;
            sprintf(errorHandler.msg,"!03,TForwardInside\r\n");
            errorHandler.setInfo();
        }
    }


    virtual NodeStatus onUpdate(Blackboard& bb) {

        errorHandler.setError("!03,TForwardInside  was called\r\n");

        // Wenn zeit in state > 10 Sek dann fehler oder anhalten undnochmal checken.
        if(getTimeInNode()> 6000) {
            errorHandler.setError("!03,TForwardInside  too long in state\r\n");
        }

        if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
            return BH_SUCCESS;
        }

        if(getTimeInNode()> 3000 ) { // Falls nach 3 Sekunden noch nicht inside, dann Kurve einleiten

            if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightOutside()) {
                bb.motor.L->setSpeed((bb.cruiseSpeed-13));
                bb.motor.R->setSpeed((bb.cruiseSpeed+13));
                bb.driveDirection = DD_FORWARD_INSIDE;
                sprintf(errorHandler.msg,"!03,TForwardInside > 3000\r\n");
                errorHandler.setInfo();
            }
            if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside()) {
                bb.motor.L->setSpeed((bb.cruiseSpeed+13));
                bb.motor.R->setSpeed((bb.cruiseSpeed-13));
                bb.driveDirection = DD_FORWARD_INSIDE;
                sprintf(errorHandler.msg,"!03,TForwardInside > 3000\r\n");
                errorHandler.setInfo();
            }
        }
        return BH_RUNNING;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {

    }

};


class TRotateX: public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

    TRotateX ()  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
        bb.motor.turnTo(bb.arcRotateXArc,bb.cruiseSpeed);
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,TRotateX  too long in state\r\n");
        }

        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }
        return BH_RUNNING;
    }
};

class TSetArc90CW: public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

    TSetArc90CW ()  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.arcRotateXArc = 90;
        bb.driveDirection = DD_ROTATECW;
        //debug->printf("bb.arcRotateXArc = 90\r\n");
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        return BH_SUCCESS;
    }

};


class TSetflagForceSmallRotAngle: public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

    TSetflagForceSmallRotAngle ()  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.flagForceSmallRotAngle = 2;
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        return BH_SUCCESS;
    }
};

class TSetArc90CC: public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

    TSetArc90CC ()  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.arcRotateXArc = -90;
        bb.driveDirection =  DD_ROTATECC;
        //debug->printf("bb.arcRotateXArc = -90\r\n");
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        return BH_SUCCESS;
    }

};


class TSetArc50CW: public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

    TSetArc50CW ()  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.arcRotateXArc = 50;
        //debug->printf("bb.arcRotateXArc = 50\r\n");
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        return BH_SUCCESS;
    }

};


class TSetArc50CC: public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

    TSetArc50CC ()  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.arcRotateXArc = -50;
        //debug->printf("bb.arcRotateXArc = -50\r\n");
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        return BH_SUCCESS;
    }

};

#endif