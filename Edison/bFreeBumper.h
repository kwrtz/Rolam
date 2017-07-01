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

#ifndef BH_BUMPER_H
#define BH_BUMPER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"


class THardstop: public Node
{
private:

public:

    THardstop() {}

    virtual void onInitialize(Blackboard& bb) {
        bb.motor.hardStop();
    }


    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.motor.isStopped()) { // Warten bis motor gestoppt
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }
};


class TFreeBumper: public Node
{
private:
    int counter,counter1;
    unsigned long lastTimeCalled;
public:

    TFreeBumper():counter(0), counter1(0),lastTimeCalled(0)  {}

    virtual void onInitialize(Blackboard& bb) {

        bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

        bb.flagPerimeterStateChanged =  PSC_NONE;

        counter = 0;

        // Abfangen, wenn bumperereignis wie bei Areax und Perimetertracking nicht weiter bearbeitet wird. Dann fährt robbi immer wieder gegen Hindernis. 
        if(millis() - lastTimeCalled < 5000){
           counter1++;
        }    
        else{
          counter1 = 0;    
        }
        if(counter1 > 5){
           errorHandler.setError("!03,freeBumper counter1 > 5\r\n");
        }    

        if (bb.perimeterSensoren.isLeftOutside() ||  bb.perimeterSensoren.isRightOutside()) {
            bb.flagBumperOutsidePerActivated  = true;
            bb.flagBumperInsidePerActivated  = false;
            sprintf(errorHandler.msg,"!03,*flagBumperOutsidePerActivated=true dd was: %s\r\n",enuDriveDirectionString[bb.driveDirection]);
            errorHandler.setInfo();
        } else {
            bb.flagBumperOutsidePerActivated  = false;
            bb.flagBumperInsidePerActivated  = true;
            sprintf(errorHandler.msg,"!03,*flagBumperInsidePerActivated=true  dd was: %s\r\n",enuDriveDirectionString[bb.driveDirection]);
            errorHandler.setInfo();
        }

        switch(bb.driveDirection) {
            case DD_FORWARD:
            case DD_OVERRUN:
                bb.motor.setSpeed(-bb.cruiseSpeed);
                break;
            case DD_FORWARD20:
            case DD_FORWARD_INSIDE:
                bb.motor.rotateAngle(-20,bb.cruiseSpeed);
                break;
            case DD_REVERSE_LINE_FOLLOW:
            case DD_REVERSE_INSIDE:
                bb.motor.rotateAngle(20,bb.cruiseSpeed);
                break;
            case DD_REVERSE_ESC_OBST:
                bb.motor.setSpeed(bb.cruiseSpeed);
                //bb.motor.rotateAngle(25,bb.cruiseSpeed);
                break;
            case DD_ROTATECW:
                bb.motor.turnTo(-30,bb.cruiseSpeed);
                break;
            case DD_ROTATECC:
                bb.motor.turnTo(30,bb.cruiseSpeed);
                break;
            case DD_SPIRAL_CW:
                bb.cruiseSpeed = bb.CRUISE_SPEED_MEDIUM;
                bb.motor.L->setSpeed(-1*bb.CRUISE_SPEED_MEDIUM);
                bb.motor.R->setSpeed(-5);
                break;    
        }

    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        int buffer;

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,freeBumper  too long in state\r\n");
        }

        if(getTimeInNode()> 3000) { // Nach 3 sek sollte bumper frei sein
            if(counter < 3) {
                if (bb.bumperSensor.isBumperActivated()) {   //wurde bumper bei berfreiung wieder betätigt? Dann wurde Motor auch hard gestoppt von sensor und man muss erneut aktiviert werden. Max. 3x
                    sprintf(errorHandler.msg,"!03,FreeBumper Bumper bei Befreiung erneut betaetigt\r\n");
                    errorHandler.setInfo();
                    buffer = counter; // Save counter value in order it will be reset in onInitialize
                    onInitialize(bb);
                    counter = buffer;
                    setTimeInNode(millis());
                    counter++;
                }
            }
            if(counter >= 3) {
                errorHandler.setError("!03,freeBumper  counter >= 3\r\n");
            }
        }



        switch(bb.driveDirection) {
            case DD_FORWARD:
            case DD_OVERRUN:
                if(!bb.bumperSensor.isBumperActivated() &&  getTimeInNode() > 1000) {
                    return BH_SUCCESS;
                }
                break;
            case DD_FORWARD20:    
            case DD_FORWARD_INSIDE:
                if (bb.motor.isPositionReached()) {
                    return BH_SUCCESS;
                }
                break;
            case DD_REVERSE_LINE_FOLLOW:
            case DD_REVERSE_INSIDE:
                if (bb.motor.isPositionReached()) {
                    return BH_SUCCESS;
                }
                break;

            case DD_REVERSE_ESC_OBST:
                
                if(!bb.bumperSensor.isBumperActivated() &&  getTimeInNode() > 1000) {
                    return BH_SUCCESS;
                }
                /*
                if (bb.motor.isPositionReached()) {
                    return BH_SUCCESS;
                } */
                break;


            case DD_ROTATECW:
                if (bb.motor.isPositionReached()) {
                    return BH_SUCCESS;
                }
                break;
            case DD_ROTATECC:
                if (bb.motor.isPositionReached()) {
                    return BH_SUCCESS;
                }
                break;
             case DD_SPIRAL_CW:
                if(!bb.bumperSensor.isBumperActivated() &&  getTimeInNode() > 1500) {
                    return BH_SUCCESS;
                }
                break;             
                
            default:
                sprintf(errorHandler.msg,"!03,TFreeBumper driveDirection not found: %s",enuDriveDirectionString[bb.driveDirection]);
                errorHandler.setError();
                break;
        }

        return BH_RUNNING;
    }


    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        if(status != BH_ABORTED) {
            // Hat sich perimeterstatus geändert. von innen nach außen oder anderherum?
            if (  (bb.perimeterSensoren.isLeftOutside() ||  bb.perimeterSensoren.isRightOutside()) && bb.flagBumperInsidePerActivated ) {
                bb.flagPerimeterStateChanged = PSC_IO;
                sprintf(errorHandler.msg,"!03,flagPerimeterStateChanged i->o = true\r\n");
                errorHandler.setInfo();
            } else if (  (bb.perimeterSensoren.isLeftInside() &&  bb.perimeterSensoren.isRightInside()) && bb.flagBumperOutsidePerActivated ) {
                bb.flagPerimeterStateChanged = PSC_OI;
                sprintf(errorHandler.msg,"!03,flagPerimeterStateChanged o->i  = true\r\n");
                errorHandler.setInfo();
            }
            
            lastTimeCalled = millis();
        }
    }
};


class TselEscapeAlgorithm: public Node
{
private:
    int counter;
    const char* text;
public:

    TselEscapeAlgorithm():counter(0),text("!03,EscapeAlgorithm %s\r\n") {}


    virtual NodeStatus onUpdate(Blackboard& bb) {

        //if(bb.flagPerimeterStateChanged == PSC_NONE && bb.flagBumperInsidePerActivated) {
        if(bb.flagBumperInsidePerActivated) {
            switch(bb.driveDirection) {
                case DD_FORWARD:
                case DD_OVERRUN://Kann hier normal nicht auftreten
                case DD_SPIRAL_CW:
                    bb.flagEscabeObstacleConFlag = FEO_BACK180;
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_FORWARD_INSIDE: //Kann hier nicht auftreten
                    errorHandler.setError("!03,DD_FORWARD_INSIDE in Bumper aktiviert 1\r\n");
                    bb.flagEscabeObstacleConFlag = FEO_BACK180;
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setError();
                    break;
                case DD_FORWARD20:
                    bb.flagEscabeObstacleConFlag = FEO_ROT;
                    bb.flagForceSmallRotAngle=4;
                    errorHandler.setInfo("!03,bb.flagForceSmallRotAngle=4\r\n");
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setError();
                    break;
                case DD_REVERSE_INSIDE:         //Kann auftreten wenn reverse inside gerade in die Schleife gefahren ist
                    bb.flagEscabeObstacleConFlag = FEO_ROT;
                    bb.flagForceSmallRotAngle=4;
                    errorHandler.setInfo("!03,bb.flagForceSmallRotAngle=4\r\n");
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_REVERSE_ESC_OBST: // Wenn bei Zuruecksetzen Obstacle gerammt wird
                    bb.flagEscabeObstacleConFlag = FEO_FWD20;
                    bb.flagForceSmallRotAngle=4; // 4x kleinen winkel drehen.
                    errorHandler.setInfo("!03,bb.flagForceSmallRotAngle=4\r\n");
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_ROTATECW:
                    bb.flagEscabeObstacleConFlag = FEO_ROTCC;
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_ROTATECC:
                    bb.flagEscabeObstacleConFlag = FEO_ROTCW;
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_REVERSE_LINE_FOLLOW: // Aktuell nicht behandelt da voraussetung, kein Hinderniss auf Perimeter 
                    bb.flagBumperOutsidePerActivated  = false;
                    bb.flagBumperInsidePerActivated  = false;
                    break;                    
                    
                default:
                    sprintf(errorHandler.msg,"!03,TchangeDriveDirection driveDirection not found: %s\r\n",enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setError();
                    break;
            }
        }


        //if(bb.flagPerimeterStateChanged == PSC_NONE && bb.flagBumperOutsidePerActivated) {
        if(bb.flagBumperOutsidePerActivated) {

            switch(bb.driveDirection) {
                case DD_FORWARD:
                case DD_OVERRUN:
                case DD_SPIRAL_CW:
                    bb.flagEscabeObstacleConFlag = FEO_BACKINSIDE;
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_FORWARD_INSIDE:
                    errorHandler.setError("!03,DD_FORWARD_INSIDE in Bumper aktiviert 2\r\n");
                    bb.flagEscabeObstacleConFlag = FEO_BACK180;
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setError();
                    break;
                case DD_FORWARD20:
                    if ( bb.perimeterSensoren.isLeftOutside()) {
                        bb.flagEscabeObstacleConFlag = FEO_ROTCC;
                    } else if( bb.perimeterSensoren.isRightOutside()) {
                        bb.flagEscabeObstacleConFlag = FEO_ROTCW;
                    } else {
                        bb.flagEscabeObstacleConFlag = FEO_ROT;
                    }
                    bb.flagForceSmallRotAngle=4;
                    errorHandler.setInfo("!03,bb.flagForceSmallRotAngle=4\r\n");
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setError();
                    break;
                case DD_REVERSE_INSIDE:
                    if ( bb.perimeterSensoren.isLeftOutside()) {
                        bb.flagEscabeObstacleConFlag = FEO_ROTCC;
                    } else if( bb.perimeterSensoren.isRightOutside()) {
                        bb.flagEscabeObstacleConFlag = FEO_ROTCW;
                    } else {
                        bb.flagEscabeObstacleConFlag = FEO_ROT;
                    }
                    
                    bb.flagForceSmallRotAngle=5;
                    errorHandler.setInfo("!03,bb.flagForceSmallRotAngle=5\r\n");
                    
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_REVERSE_ESC_OBST:
                    // Kann eigentlich nicht auftreten. Wenn DD_REVERSE_ESC_OBST inside aktiviert wurde, wird Eriegnis dass in Schleife ist, vorher von  TConPerOutside abgefangen.
                    // Kann möglicherweise auftreten, wenn rückwärts ausgewichen nahe am Perimeter und backcoil immernoch innerhalb ist.
                    // Einfach nur drehen. Dann normal weitermachen. Dazu muss dann nach rotieren driveDirection geändert werden.
                    if ( bb.perimeterSensoren.isLeftOutside()) {
                        bb.flagEscabeObstacleConFlag = FEO_ROTCC;
                    } else if( bb.perimeterSensoren.isRightOutside()) {
                        bb.flagEscabeObstacleConFlag = FEO_ROTCW;
                    } else {
                        bb.flagEscabeObstacleConFlag = FEO_ROT;
                    }
                    
                    bb.flagForceSmallRotAngle=5; // 4x kleinen winkel drehen.
                    errorHandler.setInfo("!03,bb.flagForceSmallRotAngle=5\r\n");
                    
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_ROTATECW:
                    bb.flagEscabeObstacleConFlag = FEO_BACKINSIDE;
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_ROTATECC:
                    bb.flagEscabeObstacleConFlag = FEO_BACKINSIDE;
                    sprintf(errorHandler.msg,text,enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setInfo();
                    break;
                case DD_REVERSE_LINE_FOLLOW: // Aktuell nicht behandelt da voraussetung, kein Hinderniss auf Perimeter
                    bb.flagBumperOutsidePerActivated  = false;
                    bb.flagBumperInsidePerActivated  = false;
                    break;        
                default:
                    sprintf(errorHandler.msg,"!03,TchangeDriveDirection driveDirection not found: %s\r\n",enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
                    errorHandler.setError();
                    break;
            }
        }

        /*
                //////////////////////////////////////
                if(bb.flagPerimeterStateChanged == PSC_IO) {

                    // Nur bei wechsel von inside nach outside richtung anpassen
                    // Coils waren bei bumperereignis innen und sind nun durch befreieung außerhalb

                    bb.flagBumperOutsidePerActivated  = true; // OK, ich bin nun ausßerhalb und so tun als ob ereignis außerhalb aufgetreten ist.
                    bb.flagBumperInsidePerActivated  = false;

                    switch(bb.driveDirection) {
                        case DD_FORWARD:
                        case DD_OVERRUN:
                            bb.driveDirection = DD_REVERSE_ESC_OBST;
                            sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
                            errorHandler.setInfo();
                            break;
                        case DD_FORWARD_INSIDE:
                            bb.driveDirection = DD_REVERSE_ESC_OBST;
                            sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
                            errorHandler.setInfo();
                            break;

                        case DD_REVERSE_INSIDE_CURVE_CW:
                        case DD_REVERSE_INSIDE_CURVE_CC:
                        case DD_REVERSE_INSIDE:
                            bb.driveDirection = DD_REVERSE_ESC_OBST;
                            sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
                            errorHandler.setInfo();
                            break;
                        case DD_REVERSE_ESC_OBST:
                            bb.driveDirection = DD_OVERRUN;
                            sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
                            errorHandler.setInfo();
                            break;
                        case DD_ROTATECW:
                            bb.driveDirection = DD_ROTATECC;
                            sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
                            errorHandler.setInfo();
                            break;
                        case DD_ROTATECC:
                            bb.driveDirection = DD_ROTATECW;
                            sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
                            errorHandler.setInfo();
                            break;
                        default:
                            sprintf(errorHandler.msg,"!03,driveDirection not found: %s",enuDriveDirectionString[bb.driveDirection]);
                            errorHandler.setError();
                            break;
                    }
                }
                //////////////////////////////////////

        */

        return BH_SUCCESS;
    }


};



#endif