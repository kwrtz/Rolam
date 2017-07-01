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
#ifndef BH_CRUISE_H
#define BH_CRUISE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "tables.h"
#include "Blackboard.h"



//forums.parallax.com/discussion/154274/the-artist-robot/p5
class TCruiseSpiral: public Node    // Each task will be a class (derived from Node of course).
{
private:

    uint32_t lastTimeExecuted;

    float speedOuter;
    float speedInner;

    float waitUntilOuterCmReached;

    float radius;
    float addRadius;


public:

    TCruiseSpiral () {

    }

    virtual void onInitialize(Blackboard& bb) {

        addRadius = 17.0f/GETTF(TF_SPIRAL_SEGMENTS); // Radius that should be increased in one round divided by 16 because every 22° the speed and cm will calculated new. I chose 35.5/2=17 in because the cutter disc should overlap each round.

        radius = GETTF(TF_START_SPIRAL_RADIUS_CM); //22.0f; // Start with bigger radius in order not to stall the inner wheel 35.5

        /*
        sprintf(errorHandler.msg,"!03,addRadius = %f\r\n",addRadius);
        errorHandler.setInfo();
        sprintf(errorHandler.msg,"!03,radius = %f\r\n",radius);
        errorHandler.setInfo();
        */
        
        calcSpeed(radius,bb);
        calcOuterCm(radius);
        bb.motor.startDistanceMeasurementSpiral();
        bb.motor.L->setSpeed(speedOuter);
        bb.motor.R->setSpeed(speedInner);
        bb.driveDirection = DD_SPIRAL_CW;
         
    }

    void calcSpeed( float _r,Blackboard& bb) {
        float d = GETTF(TF_DISTANCE_BETWEEN_WHEELS_CM);
        
        speedOuter = mapf(_r,  GETTF(TF_START_SPIRAL_RADIUS_CM) , GETTF(TF_MAX_SPIRAL_RADIUS_CM) , bb.CRUISE_SPEED_MEDIUM, bb.CRUISE_SPEED_HIGH);
        bb.cruiseSpeed= speedOuter;
        speedInner= speedOuter * (2*_r-d) / (2*_r+d);
    }

    void calcOuterCm( float _r) {
        float d = GETTF(TF_DISTANCE_BETWEEN_WHEELS_CM);
        waitUntilOuterCmReached = (PI * (2*_r+d))/GETTF(TF_SPIRAL_SEGMENTS); // cm for 360 degree divided by 16 to get cm for 22,5 degrees 
    } 

    virtual NodeStatus onUpdate(Blackboard& bb) {
        // get the driven distance: bb.motor.getDistanceLInCM() and check if it has driven the calculated way: waitUntilOuterCmReached
        if (bb.motor.getDistanceLInCMSpiral() > waitUntilOuterCmReached) {
            bb.motor.startDistanceMeasurementSpiral();
            radius +=addRadius;
            //sprintf(errorHandler.msg,"!03,radius +=addRadius %f\r\n",radius);
            //errorHandler.setInfo();
            calcSpeed(radius,bb);
            calcOuterCm(radius);
            bb.motor.L->setSpeed(speedOuter);
            bb.motor.R->setSpeed(speedInner);
        }

        if(radius > GETTF(TF_MAX_SPIRAL_RADIUS_CM) ) { // Stop Spiral after 150cm radius
            /*
            sprintf(errorHandler.msg,"!03,r: %f max: %f\r\n",radius, GETTF(TF_MAX_SPIRAL_RADIUS_CM));
            errorHandler.setInfo();
            errorHandler.setInfo("TCruiseSpiral end BH_SUCCESS\r\n");
            */
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        bb.flagCruiseSpiral = false;
        bb.lastTimeSpiralStarted = millis();
        /*
        errorHandler.setInfo("TCruiseSpiral ot diable flagCruiseSpiral\r\n");
        
        if (status == BH_ABORTED) {
            errorHandler.setInfo("TCruiseSpiral BH_ABORTED\r\n");
        } 
        */   
    }

};


class TCruiseBatLow: public Node
{
private:

public:

    TCruiseBatLow() {

    }

    virtual void onInitialize(Blackboard& bb) {

    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        bb.flagGoHome = false;
        bb.setBehaviour(BH_FINDPERIMETER);
        //errorHandler.setInfo("!03,bb.setBehaviour(BH_FINDPERIMETER);");
        return BH_SUCCESS;
    }
};


class TCruiseToPerimeter: public Node    // Each task will be a class (derived from Node of course).
{
private:
    unsigned long lastTimeIn;
public:

    TCruiseToPerimeter() : lastTimeIn(0) {}


    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
            bb.motor.stop();
            bb.cruiseSpeed = 0;
            return BH_SUCCESS;
        }


        if (millis() - lastTimeIn > 100ul) { // Alle 100ms geschwindigkeit einstellen
            lastTimeIn = millis();
            bb.motor.setSpeed(bb.cruiseSpeed);
            bb.driveDirection = DD_FORWARD;
        }

        return BH_RUNNING;
    }
};

class TCruiseStopped: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TCruiseStopped() {}


    virtual NodeStatus onUpdate(Blackboard& bb) {


        if (bb.motor.isStopped()) {
            bb.cruiseSpeed = 0;
            bb.driveDirection = DD_FORWARD;
            //errorHandler.setInfo("!03,TCruiseStopped SUCCESS\r\n" );

            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }
};

class TCruiseRotCW: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TCruiseRotCW() {}

    virtual void onInitialize(Blackboard& bb) {
        bb.cruiseSpeed = bb.CRUISE_ROTATE_LOW;
        bb.motor.turnTo(380,bb.cruiseSpeed);
        bb.driveDirection =  DD_ROTATECW;
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {


        if (bb.perimeterSensoren.isBackOutside()) {
            //bb.motor.stopPositioning();
            bb.motor.stop();
            bb.cruiseSpeed = 0;
            bb.driveDirection = DD_ROTATECW;
            bb.setBehaviour(BH_PERITRACK);
            //errorHandler.setInfo("!03,bb.setBehaviour(BH_PERITRACK);\r\n");
            return BH_SUCCESS;
        }


        if(bb.motor.isPositionReached()) {
            errorHandler.setError("!03,CruiseRotLeft line not found\r\n");
            return BH_FAILURE;
        }

        return BH_RUNNING;
    }
};


class TCruiseSpeedToMotor: public Node    // Each task will be a class (derived from Node of course).
{
private:
    unsigned long lastTimeIn;
public:

    TCruiseSpeedToMotor () : lastTimeIn(0) {}

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if (millis() - lastTimeIn > 100ul) { // Alle 100ms geschwindigkeit einstellen
            lastTimeIn = millis();
            bb.motor.setSpeed(bb.cruiseSpeed);
            bb.driveDirection = DD_FORWARD;
        }
        return BH_RUNNING;
    }
};


class TCruiseObstacleNear: public Node    // Each task will be a class (derived from Node of course).
{
private:
    unsigned long lastTimeMotorDecceleration;
public:

    TCruiseObstacleNear () : lastTimeMotorDecceleration(0) {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.rangeSensor.isNearObstacle()) {
            if (millis() - lastTimeMotorDecceleration > 100ul) { // Alle 100ms geschwindigkeit senken
                lastTimeMotorDecceleration = millis();

                // geschwindigkeit absenken auf obstacle speed
                if (bb.cruiseSpeed > bb.CRUISE_SPEED_OBSTACLE)
                    bb.cruiseSpeed -= 6; //5
                else {
                    bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
                }
                bb.timeCruiseSpeedSet = millis();
                //errorHandler.setInfo("!03,TCruiseObstacleNear\r\n");
            }
        }

        return BH_FAILURE;
    }
};

class TCruisePerimeterNear: public Node    // Each task will be a class (derived from Node of course).
{
private:
    unsigned long lastTimeMotorDecceleration;
public:

    TCruisePerimeterNear  () : lastTimeMotorDecceleration(0) {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(GETTB(TB_ACTVATE_AUTO_SPIRAL)) {
            if(bb.mowMotorSensor.motorUnderHeavyLoad) {
                if(millis()-bb.lastTimeSpiralStarted  > 15000ul) {
                    bb.flagCruiseSpiral = true;
                }
            }
            if( (millis()-bb.lastTimeSpiralStarted  > 180000ul) &&    bb.mowMotorSensor.checkIfUnderLoad() ) { // Alle 3 Minuten Spirale starten wenn motor belastet
                bb.flagCruiseSpiral = true;
            }
        }

        if (bb.perimeterSensoren.isNearPerimeter() || bb.mowMotorSensor.motorUnderHeavyLoad) {
            if (millis() - lastTimeMotorDecceleration > 100ul) { // Alle 100ms geschwindigkeit senken
                lastTimeMotorDecceleration = millis();

                // geschwindigkeit absenken auf low speed
                if (bb.cruiseSpeed > bb.CRUISE_SPEED_MEDIUM)
                    bb.cruiseSpeed -= 6;
                else {
                    bb.cruiseSpeed = bb.CRUISE_SPEED_MEDIUM;
                }
                bb.timeCruiseSpeedSet = millis();
                //errorHandler.setInfo("!03,TCruisePerimeterNear\r\n");
            }

        }

        return BH_FAILURE;
    }
};


class TCruiseHighSpeed: public Node    // Each task will be a class (derived from Node of course).
{
private:
    unsigned long lastTimeMotorAcceleration;
public:

    TCruiseHighSpeed  () : lastTimeMotorAcceleration(0) {}

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if (millis()-bb.timeCruiseSpeedSet > 1000) { // After 1 Seconds set accelerate cruise speed to high
            if (millis() - lastTimeMotorAcceleration > 100) { // Alle 100ms geschwindigkeit erhoehen
                lastTimeMotorAcceleration = millis();

                bb.cruiseSpeed += 1;

                if (bb.cruiseSpeed < bb.CRUISE_SPEED_OBSTACLE) {
                    bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
                }
                if (bb.cruiseSpeed > bb.CRUISE_SPEED_HIGH) {
                    bb.cruiseSpeed = bb.CRUISE_SPEED_HIGH;
                }
                //errorHandler.setInfo("!03,TCruiseHighSpeed\r\n");

            }
        }
        return BH_FAILURE;
    }
};




class TCruiseStartMowMotor: public Node
{
private:
    unsigned long startTime;
public:

    TCruiseStartMowMotor() {

    }

    virtual void onInitialize(Blackboard& bb) {
        if( !bb.motor.isMowMotRunning()) {
            bb.motor.mowMotStart();
            bb.motor.stop(); // Drivemotors must be stopped. Normally they are, but here only for safety.
            startTime = millis();
        } else {
            startTime =  millis() - 10000ul;
        }
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        // Wait 5 Sekonds until mow motor is running full speed
        if( millis()- startTime < 5000ul) {
            return BH_RUNNING;
        }

        // Set starttime in order that if( millis()- startTime < 5000ul) will not called again. Also not when millis overrun.
        // The mow mnotor is stopped in bb.setBehaviour()
        startTime =  millis() - 10000ul;
        return BH_FAILURE;

    }
};




#endif