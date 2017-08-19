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
#ifndef BH_FOLLOWLINE_H
#define BH_FOLLOWLINE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "PID_v2.h"



/*
class TconStopOvershootLeft: public Node    // Condition
{
private:

public:

    TconStopOvershootLeft() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(bb.perimeterSensoren.isBackInside()) {
            if ( (millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB) > 10000 ) { // Links  ueber Rand
                return BH_SUCCESS;
            }
        }

        return BH_FAILURE;
    }
};


class TconStopOvershootRight: public Node    // Condition
{
private:

public:

    TconStopOvershootRight() {}

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(bb.perimeterSensoren.isBackOutside()) {
            if ( (millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB) > 10000 ) { // Rechts  ueber Rand
                return BH_SUCCESS;
            }
        }

        return BH_FAILURE;
    }
};

*/

class TperTrackChargingStationReached: public Node
{
private:

public:

    TperTrackChargingStationReached() {


    }

    virtual void onInitialize(Blackboard& bb) {
        bb.motor.enableDefaultRamping();

    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        bb.setBehaviour(BH_CHARGING);
        //debug->printf("bb.setBehaviour(BH_CHARGING);");
        return BH_SUCCESS;
    }
};



class TFLRotateCC: public Node
{
private:

public:

    TFLRotateCC() {

    }

    virtual void onInitialize(Blackboard& bb) {
        bb.cruiseSpeed = 15;
        bb.driveDirection = DD_ROTATECC;
        bb.motor.L->setSpeed(-15);
        bb.motor.R->setSpeed(15);
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,FLRotateLeft too long in state\r\n");
        }

        if (bb.perimeterSensoren.isBackInside()) { // Spule ueber Fahrspurmittelpunkt
            return BH_SUCCESS;
        }



        return BH_RUNNING;
    }
};


class TFLRotateCW: public Node
{
private:

public:

    TFLRotateCW() {

    }

    virtual void onInitialize(Blackboard& bb) {
        bb.cruiseSpeed = 15;
        bb.driveDirection = DD_ROTATECW;
        bb.motor.L->setSpeed(15);
        bb.motor.R->setSpeed(-15);
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,FLRotateRight too long in state\r\n");
        }


        if (bb.perimeterSensoren.isBackOutside()) {  // Spule Ã¼ber Fahrbahnmitte nach innen
            return BH_SUCCESS;
        }


        return BH_RUNNING;
    }
};

/*
  FÃ¼r das finden eines Dreiecks in der Schleife wird stÃ¤ndig nach einer Linkskurve "Ausschau" gehalten.
  Dazu werden alle 500ms die Encoderdaten ausgelesen und ausgewertet.
  Wenn eine Linkskurve erkannt wurde wird die Zeit festgehaten und timeLeftTurnFound gesetzt.
  Wenn ein Dreieck vohanden ist, wird der Robbi nicht mehr so einfach von innen nach auÃŸen wechseln kÃ¶nnen,
  da er nach der scharfen Linkskurfe bereits Ã¼ber das Ende des Dreieckes gefahren ist. Daher wird er auf jeden Fall in die if Abfrage:
  "if ( (millis()-lastTransitionTime) > 3500 )" gehen wo er rotiert. Hier wird geguckt, ob die Linkskurve innerhalb
  der letzten 5 Sek. gefunden wurde. Wenn ja, wird flagTriangleFound = true gesetzt und rotiert, bis die andere Seite des Dreiecks erreicht
  wurde. Wenn diese erreicht wurde, geht die Spule von innen nach auÃŸen. Bei diesem Ãœbergang wird geprÃ¼ft ob ein Dreieck gefunden wurde mit
  flagTriangleFound. Weiterhin muss mindesten noch eine Kurve von 80 nach rechts gefahren worden sein und die  Zeit fÃ¼r die Rechtskurve muss mindestens
  4Sek. gedauert haben. Dann werden die Zweige im BehaviourTree mit  bb.flagFollowLine = false;  bb.flagGoHome = true; umgeschaltet, so dass der Mower wieder normal fÃ¤hrt
  bis die andere Seite des Perimeters erreicht wurde.

*/

#define ENCDELTAARRAY 6  // Check curve for 3sec

class TtrackPerimeter: public Node
{
private:
    double integral;
    unsigned long lastRun;
    unsigned long lastRunCheck;
    int encDeltaL[ENCDELTAARRAY]; //Array measured encoder ticks/100ms
    int encDeltaR[ENCDELTAARRAY];
    int idxL;
    int idxR;
    unsigned long timeLeftTurnFound;
    unsigned long timeTirangleFound;
    bool flagLeftTurnFound;
    bool flagTriangleFound;
    unsigned long lastTransitionTime;
    long angle;
    long lastEncTicksL;
    long lastEncTicksR;

public:
    double Ki;

    TtrackPerimeter() {
        Ki = 0.7;
        lastRun = 0;
        integral = 0;
    }

    virtual void onInitialize(Blackboard& bb) {
        lastRun = 0;
        lastRunCheck = 0;
        integral = 0;
        memset(&encDeltaL[0], 0, ENCDELTAARRAY*sizeof(int));
        memset(&encDeltaL[0], 0, ENCDELTAARRAY*sizeof(int));
        idxL = 0;
        idxR = 0;
        lastEncTicksL = bb.encoderL.getTickCounter();
        lastEncTicksR = bb.encoderR.getTickCounter();
        timeLeftTurnFound = 0;
        flagLeftTurnFound = false;
        flagTriangleFound = false;
        lastTransitionTime = millis();
        angle = 0;
        bb.motor.enablePerTrackRamping();

    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        long sumL=0;
        long sumR=0;
        long buffL,buffR;

        if ( millis() - lastRun  < 100) return BH_RUNNING;
        lastRun = millis();

        // Check for left turn every 500ms.
        // Read encoder values and check for left turn.
        // If left turn found, set flagLeftTurnFound=true.
        //============================================
        if ( millis() - lastRunCheck  >= 500) {
            lastRunCheck = millis();

            buffL = bb.encoderL.getTickCounter();
            buffR = bb.encoderR.getTickCounter();
            encDeltaL[idxL++] = buffL -lastEncTicksL ;
            encDeltaR[idxR++] = buffR -lastEncTicksR;
            lastEncTicksL = buffL;
            lastEncTicksR = buffR;

            if(idxL >  ENCDELTAARRAY-1) idxL = 0;
            if(idxR >  ENCDELTAARRAY-1) idxR = 0;

            /*
                   for(int i=0; i<ENCDELTAARRAY; i++) {
                       debug->printf("%d  %d  %d\r\n",encDeltaL[i], encDeltaR[i], encDeltaL[i] - encDeltaR[i]);
                   }
                   debug->printf("==============\r\n");
            */

            for(int i=0; i<ENCDELTAARRAY; i++) {
                sumL += encDeltaL[i];
                sumR += encDeltaR[i];
            }

            angle = (sumL - sumR)/10; // Not really the correct angle but it works because the -80 and 80 for a curve found out during tests.
            //debug->printf("Winkel: %d sumL%d\r\n",angle,sumL);

            if(angle < -65 ) { // -80 found out during test by try and error
                sprintf(errorHandler.msg,"!03,Left turn found angle: %ld\r\n",angle);
                errorHandler.setInfo();
                timeLeftTurnFound = millis();
                flagLeftTurnFound = true;
            }
        }
        // Calculate Speed
        //============================================
        double error =  bb.perimeterSensoren.magnetudeB;
        if(error <= 0) {
            error =-1;
        } else {
            error = 1;
        }

        integral = integral + (Ki*error);

        //debug->printf("error: %f\r\n",error);

        //Set integral to 0 if crossing the line
        if (sign0minus(error) != sign0minus(integral)) { //sign0minus => 0 belongs to minus
            integral = 0;
            lastTransitionTime = millis();

            // If changing form inside to outside after rotating CW and a triangle was found => cross lawn
            if(error < 0) {
                if(flagTriangleFound) {
                    if(angle > 80) { // Rotated CW
                        sprintf(errorHandler.msg,"!03,delta timeTirangleFound %lu\r\n" , millis() -  timeTirangleFound);
                        errorHandler.setInfo();
                        if( millis()-timeTirangleFound > 4300) { // Rotation time needed minimum 3.5Seconds to reach other side of triangle //3500
                            sprintf(errorHandler.msg,"!03,Cross Lawn Activated\r\n");
                            errorHandler.setInfo();
                            bb.setBehaviour(BH_FINDPERIMETER);
                            //debug->printf("bb.setBehaviour(BH_FINDPERIMETER);\r\n");
                            flagTriangleFound = false;
                        } else {
                            flagTriangleFound = false;
                        }
                    }
                }
            }
        }

        //debug->printf("trans: %lu\r\n",millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB);

        double Output = integral ;

        //debug->printf("p:%f i%f d:%f o:%f\r\n",Kp * error , integral, Kd * derivate, Output);

        if(Output > bb.LINEFOLLOW_SPEED_LOW) Output = bb.LINEFOLLOW_SPEED_LOW;
        if(Output < -1*bb.LINEFOLLOW_SPEED_LOW) Output = -1*bb.LINEFOLLOW_SPEED_LOW;


        bb.cruiseSpeed = bb.LINEFOLLOW_SPEED_LOW;
        bb.driveDirection = DD_REVERSE_LINE_FOLLOW;

        if(error>0.0f) { //Set Speed Inside Perimeter and Check for Triangle if more than 3.3 seconds inside.
            if ( (millis()-lastTransitionTime) > 3300 ) { // If more than 3.5sec inside rotate full
                bb.cruiseSpeed = 15;
                bb.driveDirection = DD_ROTATECW;
                bb.motor.L->setSpeed(15);
                bb.motor.R->setSpeed(-15);

                // Check for triangle
                // If the coil is more than 3.5 seconds inside perimeter, then we found a sharp corner
                // If there was a left turn in the last 5 seconds, we found a triangel
                if(flagLeftTurnFound) {
                    if (millis() -  timeLeftTurnFound < 5000) {
                        sprintf(errorHandler.msg,"!03,TRIANGLE FOUND deltaTime%lu\r\n" , millis() -  timeLeftTurnFound);
                        errorHandler.setInfo();
                        timeTirangleFound = millis();
                        flagTriangleFound = true;
                        flagLeftTurnFound = false;
                    } else {
                        flagTriangleFound = false;
                        flagLeftTurnFound = false;
                    }
                }


            } else if ( (millis()-lastTransitionTime) > 2800 ) { // If more than 2.8sec inside rotate more aggressive
                bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
                bb.motor.R->setSpeed(-(bb.cruiseSpeed+10));
            } else if ( (millis()-lastTransitionTime) > 2000 ) { // If more than 2sec inside rotate aggressive
                bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
                bb.motor.R->setSpeed(-(bb.cruiseSpeed+5));
            } else {
                bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
                bb.motor.R->setSpeed(-(bb.cruiseSpeed));
            }

        } else { //Set Speed Outside Perimeter

            if ( (millis()-lastTransitionTime) > 1800 ) { // // If more than 2sec outside rotate full
                bb.cruiseSpeed = 15;
                bb.driveDirection = DD_ROTATECC;
                bb.motor.L->setSpeed(-15);
                bb.motor.R->setSpeed(15);
            } else if ( (millis()-lastTransitionTime) > 1500 ) { // If more than 1.5sec outside rotate more aggressive
                bb.motor.L->setSpeed(-(bb.cruiseSpeed+10));
                bb.motor.R->setSpeed(-(-15));
            } else if ( (millis()-lastTransitionTime) > 1000 ) { // If more than 1sec outside rotate aggressive
                bb.motor.L->setSpeed(-(bb.cruiseSpeed+10));
                bb.motor.R->setSpeed(-(-8));
            } else {
                bb.motor.L->setSpeed(-(bb.cruiseSpeed+15)); //5
                bb.motor.R->setSpeed(-(bb.cruiseSpeed -25)); //30

            }
        }

        if ( (millis()-lastTransitionTime) > 12000 ) {
            errorHandler.setError("!03,TtrackPerimeter line crossing > 12000\r\n");
        }

        return BH_RUNNING;
    }


    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        //debug->printf("onTerminate TFLfollowLine enableDefaultRamping()\r\n" );
        //bb.motor.enableDefaultRamping();
    }

};


class TMotorStopFast: public Node
{
private:

public:

    TMotorStopFast() {}

    virtual void onInitialize(Blackboard& bb) {
        bb.motor.enableFastStopRamping();
        bb.motor.stop();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,motorStop too long in state\r\n");
        }

        if (bb.motor.isStopped()) {
            bb.cruiseSpeed = 0;
            //debug->printf("onUpdate enableFastStopRamping()\r\n");
            //bb.motor.enableFastStopRamping();
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        //debug->printf("onTerminate FastStopRamping()\r\n");
        //bb.motor.enableDefaultRamping();
    }
};

/**********************************
class TFLfollowLine: public Node
{
private:

    double Setpoint, Amplitude, Output; // Pid setpoint, input und oputput
    double last_error, integral;
    int counter;

public:
    //PIDPOS myPID;
    double Kp, Ki, Kd;
    unsigned long nextTimeMotorPerimeterControl;

    TFLfollowLine() {
        Kp = 0.0021f; //0.003f 21
        Ki = 0.00013f; //0.00004f;0.000168f000247
        Kd = 0.00446f; //0.003f0.0065625f
        Setpoint = 0.0f;
        Amplitude = 0.0f;
        Output = 0.0f; // Pid setpoint, input und oputput

        nextTimeMotorPerimeterControl = 0;
        last_error=0;
        integral = 0;
    }

    virtual void onInitialize(Blackboard& bb) {
        Output = 0;
        Setpoint = bb.perimeterSensoren.magLineFollow;
        //myPID.Initialize();
        nextTimeMotorPerimeterControl = 0;
        last_error=0;
        integral = 0;
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if (millis() < nextTimeMotorPerimeterControl) return BH_RUNNING;
        nextTimeMotorPerimeterControl = millis() + 100;

        Amplitude =  bb.perimeterSensoren.magnetudeB;
        if(Amplitude < 0)
            Amplitude *= 1.34f; // 1.34f;1,5

        double error = Amplitude;
        double derivate = error-last_error;
        integral = integral + (Ki*error);

        //debug->printf("error: %f\r\n",error);


        //Set integral to 0 if crossing the line
        if (sign0minus(error) != sign0minus(integral)) {
            integral = 0;  //sign0minus => 0 belongs to minus
        }

        //     debug->printf("trans: %lu\r\n",millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB);

        //      if(integral > bb.LINEFOLLOW_SPEED_LOW) integral = bb.LINEFOLLOW_SPEED_LOW;
        //      if(integral < -1*bb.LINEFOLLOW_SPEED_LOW) integral = -1*bb.LINEFOLLOW_SPEED_LOW;

        Output = Kp  * error + integral + Kd * derivate ;


        //if(millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB > 900){
        //    Output *=1.3;
        //}


        //debug->printf("p:%f i%f d:%f ",Kp * error , integral, Kd * derivate);
        //debug->printf("output: %f\r\n",Output);

        if(Output > bb.LINEFOLLOW_SPEED_LOW) Output = bb.LINEFOLLOW_SPEED_LOW;
        if(Output < -1*bb.LINEFOLLOW_SPEED_LOW) Output = -1*bb.LINEFOLLOW_SPEED_LOW;

        last_error = error;

        bb.cruiseSpeed = bb.LINEFOLLOW_SPEED_LOW;
        bb.driveDirection = DD_REVERSE_LINE_FOLLOW;

        if(error>0.0f) {
            bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
            bb.motor.R->setSpeed(-(bb.cruiseSpeed));
        } else {
            bb.motor.L->setSpeed(-(bb.cruiseSpeed+5));
            bb.motor.R->setSpeed(-(bb.cruiseSpeed + Output));
        }


        return BH_RUNNING;
    }
};

********************************/

/**********************************
class TFLfollowLine: public Node
{
private:

    double Setpoint, Amplitude, Output; // Pid setpoint, input und oputput
    double last_error, integral;
    int amplitudenVerlauf[10];
    int counter;

public:
    //PIDPOS myPID;
    double Kp, Ki, Kd;
    unsigned long nextTimeMotorPerimeterControl;

    TFLfollowLine() {
        Kp = 18;
        Ki = 1.2;
        Kd = 0;
        Setpoint = 0.0f;
        Amplitude = 0.0f;
        Output = 0.0f; // Pid setpoint, input und oputput

        nextTimeMotorPerimeterControl = 0;
        last_error=0;
        integral = 0;
        memset(&amplitudenVerlauf[0], 0, 10*sizeof(int));
    }

    virtual void onInitialize(Blackboard& bb) {
        Output = 0;
        Setpoint = bb.perimeterSensoren.magLineFollow;
        //myPID.Initialize();
        nextTimeMotorPerimeterControl = 0;
        last_error=0;
        integral = 0;
        memset(&amplitudenVerlauf[0], 0, 10*sizeof(int));

    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if (millis() < nextTimeMotorPerimeterControl) return BH_RUNNING;
        nextTimeMotorPerimeterControl = millis() + 100;

        Amplitude =  bb.perimeterSensoren.magnetudeB;
        if(Amplitude <= 0){
            Amplitude =-1;
        }else{
            Amplitude = 1;
        }

        double error = Amplitude;
        double derivate = error-last_error;
        integral = integral + (Ki*error);

        //debug->printf("error: %f\r\n",error);


        //Set integral to 0 if crossing the line
        if (sign0minus(error) != sign0minus(integral)) {
            integral = 0;  //sign0minus => 0 belongs to minus
        }

        //     debug->printf("trans: %lu\r\n",millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB);

        //      if(integral > bb.LINEFOLLOW_SPEED_HIGH) integral = bb.LINEFOLLOW_SPEED_HIGH;
        //      if(integral < -1*bb.LINEFOLLOW_SPEED_HIGH) integral = -1*bb.LINEFOLLOW_SPEED_HIGH;

        Output = Kp  * error + integral + Kd * derivate ;


        //if(millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB > 900){
        //    Output *=1.3;
        // }



        debug->printf("p:%f i%f d:%f o:%f\r\n",Kp * error , integral, Kd * derivate, Output);


        if(Output > bb.LINEFOLLOW_SPEED_HIGH) Output = bb.LINEFOLLOW_SPEED_HIGH;
        if(Output < -1*bb.LINEFOLLOW_SPEED_HIGH) Output = -1*bb.LINEFOLLOW_SPEED_HIGH;

        last_error = error;

        bb.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
        bb.driveDirection = DD_REVERSE_LINE_FOLLOW;

        if(error>0.0f) {
            bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
            bb.motor.R->setSpeed(-(bb.cruiseSpeed));
        } else {
            bb.motor.L->setSpeed(-(bb.cruiseSpeed+5));
            //bb.motor.R->setSpeed(-(bb.cruiseSpeed -30));
            bb.motor.R->setSpeed(-(bb.cruiseSpeed -40));
        }


        return BH_RUNNING;
    }
};

*/


/**********************************


class TFLfollowLine: public Node
{
private:

    double Setpoint, Amplitude, Output; // Pid setpoint, input und oputput
    double last_error, integral;
    int amplitudenVerlauf[10];
    int counter;

public:
    //PIDPOS myPID;
    double Kp, Ki, Kd;
    unsigned long nextTimeMotorPerimeterControl;

    TFLfollowLine() {
        Kp = 0.0021f; //0.003f 21
        Ki = 0.00013f; //0.00004f;0.000168f000247
        Kd = 0.00446f; //0.003f0.0065625f
        Setpoint = 0.0f;
        Amplitude = 0.0f;
        Output = 0.0f; // Pid setpoint, input und oputput

        nextTimeMotorPerimeterControl = 0;
        last_error=0;
        integral = 0;
        memset(&amplitudenVerlauf[0], 0, 10*sizeof(int));
    }

    virtual void onInitialize(Blackboard& bb) {
        Output = 0;
        Setpoint = bb.perimeterSensoren.magLineFollow;
        //myPID.Initialize();
        nextTimeMotorPerimeterControl = 0;
        last_error=0;
        integral = 0;
        memset(&amplitudenVerlauf[0], 0, 10*sizeof(int));

    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if (millis() < nextTimeMotorPerimeterControl) return BH_RUNNING;
        nextTimeMotorPerimeterControl = millis() + 100;

        Amplitude =  bb.perimeterSensoren.magnetudeB;
        if(Amplitude < 0)
            Amplitude *= 1.5f; // 1.34f;

        double error = Amplitude;
        double derivate = error-last_error;
        integral = integral + (Ki*error);

        //debug->printf("error: %f\r\n",error);

        //memcpy(destination, source, number_of_bytes).
 //       memcpy(&amplitudenVerlauf[0],&amplitudenVerlauf[1],sizeof(int) * (10-1));
 //       amplitudenVerlauf[9]= bb.perimeterSensoren.magnetudeB;

 //        for(int i=0; i<10; i++) {
 //           debug->printf("error: %d\r\n",amplitudenVerlauf[i]);
 //       }
 //       debug->printf("==============\r\n");


 //       counter = 0;

 //       for(int i=0; i<9; i++) {
 //           if( amplitudenVerlauf[i]< 0) {
 //               if ( amplitudenVerlauf[i] < amplitudenVerlauf[i+1]) {
 //                   counter++;
 //               } else {
 //                  if(counter < 5)
 //                       counter = 0;
 //               }
 //           } else { // Positive amplitude gefunden
 //               counter = 0;
 //               break;
 //           }
 //       }

 //       debug->printf("counter: %d\r\n",counter);



        //Set integral to 0 if crossing the line
        if (sign0minus(error) != sign0minus(integral)) {
            integral = 0;  //sign0minus => 0 belongs to minus
        }

        //     debug->printf("trans: %lu\r\n",millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB);

        //      if(integral > bb.LINEFOLLOW_SPEED_HIGH) integral = bb.LINEFOLLOW_SPEED_HIGH;
        //      if(integral < -1*bb.LINEFOLLOW_SPEED_HIGH) integral = -1*bb.LINEFOLLOW_SPEED_HIGH;

        Output = Kp  * error + integral + Kd * derivate ;


        //if(millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB > 900){
        //    Output *=1.3;
        //}


        //debug->printf("p:%f i%f d:%f ",Kp * error , integral, Kd * derivate);
        //debug->printf("output: %f\r\n",Output);

        if(Output > bb.LINEFOLLOW_SPEED_HIGH) Output = bb.LINEFOLLOW_SPEED_HIGH;
        if(Output < -1*bb.LINEFOLLOW_SPEED_HIGH) Output = -1*bb.LINEFOLLOW_SPEED_HIGH;

        last_error = error;

        bb.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
        bb.driveDirection = DD_REVERSE_LINE_FOLLOW;

        if(error>0.0f) {
            bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
            bb.motor.R->setSpeed(-(bb.cruiseSpeed));
        } else {
            bb.motor.L->setSpeed(-(bb.cruiseSpeed));
            //bb.motor.R->setSpeed(-(bb.cruiseSpeed -30));
            bb.motor.R->setSpeed(-(bb.cruiseSpeed + Output));
        }


        return BH_RUNNING;
    }
};

***********************************/

#endif

