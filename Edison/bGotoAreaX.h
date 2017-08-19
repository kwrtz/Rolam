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

#ifndef BH_GOTOAREAX_H
#define BH_GOTOAREAX_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "PID_v2.h"



class TsetMowBehaviour: public Node
{
private:

public:

    TsetMowBehaviour() {
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        bb.setBehaviour(BH_MOW);
        return BH_SUCCESS;
    }
};



class TARrotate90CC: public Node
{
private:

public:

    TARrotate90CC() {

    }

    virtual void onInitialize(Blackboard& bb) {
        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
        bb.driveDirection =  DD_ROTATECC;
        bb.motor.turnTo(-1*90,bb.cruiseSpeed);
        bb.motor.enableDefaultRamping();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,TARrotate90CC too long in state\r\n");
        }

        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }
        return BH_RUNNING;

    }
};


class TGotoAreaX: public Node
{
private:
    double integral;
    unsigned long lastRun;
    unsigned long lastTransitionTime;

public:
    double Ki;

    TGotoAreaX() {
        Ki = 0.7;
        lastRun = 0;
        integral = 0;
    }

    virtual void onInitialize(Blackboard& bb) {
        lastRun = 0;
        integral = 0;
        lastTransitionTime = millis();
        bb.motor.enablePerTrackRamping();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {


        if ( millis() - lastRun  < 100) return BH_RUNNING;
        lastRun = millis();


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
        }

        //debug->printf("trans: %lu\r\n",millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB);

        double Output = integral ;

        //debug->printf("p:%f i%f d:%f o:%f\r\n",Kp * error , integral, Kd * derivate, Output);

        if(Output > bb.LINEFOLLOW_SPEED_HIGH) Output = bb.LINEFOLLOW_SPEED_HIGH;
        if(Output < -1*bb.LINEFOLLOW_SPEED_HIGH) Output = -1*bb.LINEFOLLOW_SPEED_HIGH;


        bb.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
        bb.driveDirection = DD_REVERSE_LINE_FOLLOW;

        if(error>0.0f) { //Set Speed Inside Perimeter and Check for Triangle if more than 3.5 seconds inside.
            if ( (millis()-lastTransitionTime) > 3300 ) { // If more than 3.5sec inside rotate full
                bb.cruiseSpeed = 15;
                bb.driveDirection = DD_ROTATECW;
                bb.motor.L->setSpeed(15);
                bb.motor.R->setSpeed(-15);
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
                bb.motor.L->setSpeed(-(bb.cruiseSpeed+10)); //50+10=60
                bb.motor.R->setSpeed(-(-8));  // -8
            } else {
                bb.motor.L->setSpeed(-(bb.cruiseSpeed+15)); //50+10=60
                bb.motor.R->setSpeed(-(bb.cruiseSpeed -25)); //50-25=25

            }
        }


        if ( (millis()-lastTransitionTime) > 12000 ){
            errorHandler.setError("!03,TGotoAreaX line crossing > 12000\r\n");
        }     
        return BH_RUNNING;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
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

