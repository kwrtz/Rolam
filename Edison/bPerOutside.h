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

#ifndef BH_PERIMETEROUTSIDE_H
#define BH_PERIMETEROUTSIDE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "tables.h"
#include "BehaviourTree.h"

//#define DEBUG_ROTATE_RANDOM_NUMBERS 1

#ifdef DEBUG_ROTATE_RANDOM_NUMBERS
#  define DRRN(x) x
#else
#  define DRRN(x)
#endif


#define HISTROY_BUFSIZE 3


//Structure for history and current rotation data
typedef struct  {
    long  distanceDriven; // [0] Enthaelt die gerade gefahrene distanz von der letzten rotation bis jetzt. Jedes mal nachdem rotiert wurde, wird distanzmessung neu gestartet.
    enuFlagCoilsOutside   coilFirstOutside; // [0] which coil was first outside jet
    float coilOutDifference; // [0]  Current outside differnce of both coils. When left was out 10cm and right was outside 6cm then lastCoilOutDifference = 4cm
    float coilOutAngle; // [0] Current meassured Angle of coils to Perimeter 0Â° Means Both coils parrallel outside. 90Â° Robot is parrallel to perimeter (not really measureable)

    enuDriveDirection lastRotDirection; // [0]  Enthaelt rotationsrichtung der LETZTEN Drehung.
    long lastRotAngle; // [0] EnthÃ¤lt angle der LETZTEN Drehungen
} THistory;



//############################################
class TRotateWorkx: public Node    // Each task will be a class (derived from Node of course).
{
private:
    THistory history[HISTROY_BUFSIZE];

    long randAngle; // calculated angle wich should be rotated

    int stateStuck; // State variable for stucking algorithm
    int stateAngle; //Statevariabel fÃ¼r winkelberechnung
    int memStateAngle; // Memorize state


    bool isArcNotInitialised;
    int numberArcBig;
    int numberArcSmall;
    int dodgeArcCounterBig;
    int dodgeArcCounterSmall;

    int angleCounter;


public:

    bool showValuesOnConsole;

    TRotateWorkx()  {

        // Erstmal so fÃ¼llen, das bedingungen unten nicht gleich erfÃ¼llt sind.
        for(int i=0; i< HISTROY_BUFSIZE; i++) {
            history[i].distanceDriven= 300;
            history[i].lastRotDirection = DD_ROTATECC;
            history[i].lastRotAngle = 88;
            history[i].coilFirstOutside = CO_BOTH;
            history[i].coilOutDifference = 100.0f;
            history[i].coilOutAngle = 90.0f;
        }
        stateStuck = 0;
        isArcNotInitialised=true;
        stateAngle = 0;
        angleCounter = 0;
    }

    // Shift array to right  to insert new distance at 0
    void shiftHistory() {
        for(int i=HISTROY_BUFSIZE-1; i > 0; i--) {
            history[i] = history[i-1];
        }
    }
    void addDistanceToHistory(long d) {
        history[0].distanceDriven= d;
    }
    void addDirectionToHistory(enuDriveDirection d) {
        history[0].lastRotDirection =  d;
    }
    void addAngleToHistory(long d) {
        history[0].lastRotAngle = d;
    }
    void addCoilFirstOutsideToHistory(enuFlagCoilsOutside c) {
        history[0].coilFirstOutside = c;
    }
    void addCoilOutDifferenceToHistory(float d) {
        history[0].coilOutDifference = d;
    }
    void addCoilOutAngleToHistory(float d) {
        history[0].coilOutAngle = d;
    }


    void printLastDistanceAndDirectionArray() {
        if(showValuesOnConsole == true) {

            sprintf(errorHandler.msg,"!05,distanceDriven: %ld %ld  %ld\r\n",history[0].distanceDriven,history[1].distanceDriven,history[2].distanceDriven);
            errorHandler.setInfo();

            sprintf(errorHandler.msg,"!05,coilFirstOutside %d %d %d\r\n",history[0].coilFirstOutside,history[1].coilFirstOutside,history[2].coilFirstOutside);
            errorHandler.setInfo();

            sprintf(errorHandler.msg,"!05,coilOutDifference: %f %f %f\r\n",history[0].coilOutDifference,history[1].coilOutDifference,history[2].coilOutDifference);
            errorHandler.setInfo();

            sprintf(errorHandler.msg,"!05,coilOutAngle %f %f %f\r\n",history[0].coilOutAngle,history[1].coilOutAngle,history[2].coilOutAngle);
            errorHandler.setInfo();

            sprintf(errorHandler.msg,"!05,lastRotDirection ");
            errorHandler.setInfo();
            for(int i = 0; i < HISTROY_BUFSIZE; i++) {
                if(history[i].lastRotDirection == DD_ROTATECW) {
                    sprintf(errorHandler.msg,"CW ");
                } else if (history[i].lastRotDirection == DD_ROTATECC) {
                    sprintf(errorHandler.msg,"CC ");
                } else {
                    sprintf(errorHandler.msg,"NA! ");
                }

                errorHandler.setInfo();
            }
            errorHandler.setInfo("\r\n");

            sprintf(errorHandler.msg,"!05,lastRotAngle: %ld %ld %ld\r\n",history[0].lastRotAngle, history[1].lastRotAngle, history[2].lastRotAngle);
            errorHandler.setInfo();
        }
    }


    void printNewDistanceAndArray() {

        if(showValuesOnConsole == true) {
            sprintf(errorHandler.msg,"!05,new rotDirection: ");
            errorHandler.setInfo();

            if(history[0].lastRotDirection == DD_ROTATECW) {
                sprintf(errorHandler.msg,"CW ");
            } else if (history[0].lastRotDirection == DD_ROTATECC) {
                sprintf(errorHandler.msg,"CC ");
            } else {
                sprintf(errorHandler.msg,"NA! ");
            }

            errorHandler.setInfo();
            errorHandler.setInfo("\r\n");


            sprintf(errorHandler.msg,"!05,new angle: %ld\r\n",history[0].lastRotAngle);
            errorHandler.setInfo();
        }
    }



    virtual void onInitialize(Blackboard& bb) {

        showValuesOnConsole = GETTB(TB_SHOW_ROTATE);

        // Insert data captured from last rotation to now
        addDistanceToHistory(bb.motor.getDistanceInCMForWorkx()); // Driven distance from end of last rotating until now
        addCoilFirstOutsideToHistory(bb.flagCoilFirstOutside);    // which coil was first outside jet
        addCoilOutDifferenceToHistory(bb.motor.getDistanceDiffInCMForCoilOut()); // Measured difference abs(coilL-coilR) of the way, the coils needed to go from outside to inside in cm
        addCoilOutAngleToHistory(bb.motor.getDistanceAngleCoilOut()); // Meassured Angle of coils to Perimeter 0Â° Means Both coils parrallel outside. 90Â° Robot is parrallel to perimeter (not really measureable)

        // Im Array steht nun an erster Position [0], die letzte gefahrene Distanz, der letzte gedrehte Winkel und die letzte gedrehte Richtung
        // Sowie die aktuellen Werte fÃ¼r: flagCoilFirstOutside, getDistanceDiffInCMForCoilOut, getDistanceAngleCoilOut
        // Auf Grundlage dieser Wert mÃ¼ssen nun Entscheidungen getroffen werden. Unten werden dann der neue Winkel und Winkelrichtung eingetragen, beim nÃ¤chsten Aufruf dieser
        // Funktion die dazugehÃ¶rige distanz in [0] oben eingetragen

        printLastDistanceAndDirectionArray() ;


        if(isArcNotInitialised) {
            numberArcBig = myRandom(15, 30);
            numberArcSmall  = myRandom(5, 8);
            dodgeArcCounterBig = 0;
            dodgeArcCounterSmall = 0;
            angleCounter = 0;
            DRRN(debug->printf("numberArcBig: %d;\r\n",numberArcBig);)
            DRRN(debug->printf("numberArcSmall: %d;\r\n",numberArcSmall);)
            isArcNotInitialised = false;
            memStateAngle = 0;
        }

        /**********************
        * Drehwinkel berechnen
        **********************/

        // Wenn strecke > 300cm dann standardwinkel wiederherstellen
        if(history[0].distanceDriven > 300&& stateAngle > 1) {
            stateAngle = memStateAngle;
            stateStuck = 0;
        }

        // Standarddrehwinkel festlegen
        if(stateAngle == 0 || stateAngle == 1) {  // 0 = goÃŸer Winkel, 1 = kleiner Winkel


            // Drehwinkel berechnen in AbhÃ¤ngigkeit von dodgeArcSmallCounter
            if(dodgeArcCounterBig  < numberArcBig  ) { // GroÃŸen Winkel rotieren
                dodgeArcCounterBig++;

                if(angleCounter <3) {
                    if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                        randAngle = myRandom(100, 155);  // Wenn beide Spulen drauÃŸen sind, kann Drehwinkel grÃ¶ÃŸer sein 80 155
                        errorHandler.setInfo("!05,angleCounter B <4 100-155\r\n");
                    } else {
                        randAngle = myRandom(80, 100); // Wenn eine Spule drauÃŸenist, nicht soweit drehen
                        errorHandler.setInfo("!05,angleCounter O <4 50-70\r\n");
                    }

                    angleCounter++;
                } else if(angleCounter <21) {

                    if(history[0].distanceDriven > 250 ) { // ####### cm 400

                        if (history[0].coilOutDifference < 6 && bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                            randAngle  = myRandom(90, 120); // Robbi steht fast im 0 Gradwinkel zur Schleife //Vorher 90/130
                            errorHandler.setInfo("!05,FreeAreaBig 90-120\r\n");
                        } else if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH ) {
                            randAngle = myRandom(70, 100); // Wenn beide Spulen drauÃŸen sind, kann Drehwinkel grÃ¶ÃŸer sein 80 115
                            errorHandler.setInfo("!05,FreeAreaBig 70-100\r\n");
                        } else {
                            randAngle = myRandom(50, 70); // Wenn eine Spule drauÃŸen  ist, nicht soweit drehen
                            errorHandler.setInfo("!05,FreeAreaBig 50-70\r\n"); //60 80
                        }


                    } else if(history[0].distanceDriven  > 150 ) { //cm 150

                        if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                            randAngle = myRandom(70, 100); // Wenn beide Spulen drauÃŸen sind, kann Drehwinkel grÃ¶ÃŸer sein 80 155
                            errorHandler.setInfo("!05,FreeArea 70-100\r\n");
                        } else {
                            randAngle = myRandom(50, 70); // Wenn eine Spule drauÃŸenist, nicht soweit drehen
                            errorHandler.setInfo("!05,FreeArea 50-70\r\n");
                        }

                    } else { // Korridor
                        angleCounter=0;
                        // Im Korridor nicht so stark drehen
                        if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                            randAngle = myRandom(50, 90);   // Welchen Winkel soll robbi drehen wenn zu oft ein kurzer weg gefahrebn wurde. 50/90
                        } else {
                            randAngle = myRandom(40, 60); // Wenn eine Spule drauÃŸen ist, nicht soweit drehen 40/60
                        }
                        errorHandler.setInfo("!05,Korridor\n\r");
                    }


                    angleCounter++;
                } /*else if(angleCounter <8) {
                    randAngle = myRandom(90, 115);
                    errorHandler.setInfo("!05,>90-115\r\n");
                    angleCounter++;
                } */ else if(angleCounter <22) {
                    randAngle = 90;
                    errorHandler.setInfo("!05,>90\r\n");
                    angleCounter = 0;
                }


                stateAngle = 0;

            } else { // Kleinen Winkel rotieren
                //randAngle = myRandom(50, 60);   // Normaler Drehwinkel fÃ¼r kleinen Winkel 50/60
                //errorHandler.setInfo("!05,250Area 50-60\r\n");

                if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                    randAngle = myRandom(50, 70);   // Welchen Winkel soll robbi drehen wenn zu oft ein kurzer weg gefahrebn wurde. 50/90
                    errorHandler.setInfo("!05,250/1 Area 50-70\r\n");
                } else {
                    randAngle = myRandom(30, 50);   // Wenn eine Spule drauÃŸen ist, nicht soweit drehen 40/60
                    errorHandler.setInfo("!05,250/1 Area 30-50\r\n");
                }

                dodgeArcCounterSmall++;
                stateAngle = 1;
            }

            // dodgeArcSmallCounter reseten und wieder von vorne mit groÃŸem winkel beginnen
            if (dodgeArcCounterSmall  >= numberArcSmall  ) {
                isArcNotInitialised = true;
            }
        }

        /// ####Vermutlich Ecke erwischt. Dann weiter in gleiche Richtung drehen.                                                             // Nur aktivieren wenn wenn in state 0 oder 1
        if(history[0].distanceDriven < 50 && history[1].distanceDriven > 150 &&  history[0].coilFirstOutside != history[1].coilFirstOutside && stateAngle<2) {

            if(history[0].lastRotDirection== DD_ROTATECC) {
                // Weiter CC drehen wenn vorher CC gedreht wurde
                bb.flagForceRotateDirection = FRD_CC;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,Ecke lastRotDirection[0]== DD_ROTATECC\n\r");
                }
            } else {
                // Weiter CW drehen wenn vorher CW gedreht wurde
                bb.flagForceRotateDirection = FRD_CW;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,Ecke lastRotDirection[0]== DD_ROTATECW\n\r");
                }
            }


            if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                randAngle = myRandom(70, 100); // Wenn beide Spulen drauÃŸen sind, kann Drehwinkel grÃ¶ÃŸer sein 80 155
            } else {
                randAngle = myRandom(50, 70); // Wenn eine Spule drauÃŸenist, nicht soweit drehen
            }
            errorHandler.setInfo("!05,Ecke\r\n");
        }

        // Eintrittsbedingung fÃ¼r Korridor                                   Only if big arc      latch function
        if( (history[0].distanceDriven < 150 && history[1].distanceDriven < 150  && stateAngle == 0 )  || stateAngle == 10 ) { // history[0].coilFirstOutside != history[1].coilFirstOutside &&

            if( stateAngle == 0 || stateAngle ==1) {
                memStateAngle = stateAngle;
            }
            stateAngle = 10;

            // Im Korridor nicht so stark drehen
            if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                randAngle = myRandom(50, 90);   // Welchen Winkel soll robbi drehen wenn zu oft ein kurzer weg gefahrebn wurde. 50/90
            } else {
                randAngle = myRandom(40, 60); // Wenn eine Spule drauÃŸen ist, nicht soweit drehen 40/60
            }
            errorHandler.setInfo("!05,Korridor\n\r");
        }


        // Test auf  Korridor
        // Eintrittsbedingung fÃ¼r schmalen Korridor aber nur wenn ober groÃŸer Winkel gefahren wird.
        if( (history[0].distanceDriven < 80 && history[1].distanceDriven < 80  && stateAngle == 0 )) { // history[0].coilFirstOutside != history[1].coilFirstOutside
            if( stateAngle == 0 || stateAngle ==1) {
                memStateAngle = stateAngle;
            }
            stateAngle = 30;
            errorHandler.setInfo("!05,Schmaler Korridor 1\n\r");
        }


        if( (history[0].distanceDriven < 80 && history[1].distanceDriven < 80  && history[2].distanceDriven < 80) || stateAngle == 30 ) {

            if( stateAngle == 0 || stateAngle ==1) {
                memStateAngle = stateAngle;
            }
            stateAngle = 30; // latch

            // Im schmalem Korridor nicht so stark drehen
            if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                randAngle = myRandom(50, 70);   // Welchen Winkel soll robbi drehen wenn zu oft ein kurzer weg gefahrebn wurde. 50/90
            } else {
                randAngle = myRandom(40, 50);   // Wenn eine Spule drauÃŸen ist, nicht soweit drehen 40/60
            }
            errorHandler.setInfo("!05,Schmaler Korridor 2\n\r");
        }


        if(bb.flagForceSmallRotAngle > 0) {
            randAngle = myRandom(30, 60);
            bb.flagForceSmallRotAngle--;
            errorHandler.setInfo("!05,ForceSmallRotAngle\r\n");
        }

        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;



        /**********************
        * Oszilliert Roboter an Perimeter?
        * Overrides the randAngle from above if Robot stucks
        **********************/
        // Wenn eine Strecke weniger als 20 cm gefahren wurde, dann wird angenommen, dass Ende Korridor Erreicht ist und es muss gedreht werden
        if(  (history[0].distanceDriven <20) && (stateStuck == 0) && (bb.flagCoilOutsideAfterOverrun == CO_BOTH) ) {

            if(showValuesOnConsole) {
                errorHandler.setInfo("!05,STUCK distanceDriven[0] <20\r\n");
            }
            //randAngle = 160-lastAngle[0] ;

            randAngle = 90;

            if(history[0].lastRotDirection== DD_ROTATECC) {
                // Weiter CC drehen wenn vorher CC gedreht wurde
                bb.flagForceRotateDirection = FRD_CC;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 0 lastRotDirection[0]== DD_ROTATECC\n\r");
                }
                stateStuck = 1;
            } else {
                // Weiter CW drehen wenn vorher CW gedreht wurde
                bb.flagForceRotateDirection = FRD_CW;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 0 lastRotDirection[0]== DD_ROTATECW\n\r");
                }
                stateStuck = 2;
            }
        } else if(  stateStuck == 1) { //CC gedreht
            if(history[0].distanceDriven > 40  || bb.flagCoilOutsideAfterOverrun  != CO_BOTH) {
                //Do nothing
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 1 nothing\n\r");
                }
                stateStuck = 0;
            } else {
                // Weiter CC drehen
                randAngle = 90;
                bb.flagForceRotateDirection = FRD_CC;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 1 FRD_CC\n\r");
                }
                stateStuck = 3;
            }


        } else if(  stateStuck == 2) { //CW gedreht
            if(history[0].distanceDriven > 40  || bb.flagCoilOutsideAfterOverrun  != CO_BOTH) {
                //Do nothing
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 2 nothing\n\r");
                }
                stateStuck = 0;
            } else {
                // Weiter CW drehen
                randAngle = 90;
                bb.flagForceRotateDirection = FRD_CW;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 2 FRD_CW\n\r");
                }
                stateStuck = 4;
            }



        } else if(  stateStuck == 3) { //Im State 1 wurde CC gedreht und nun ist er auf die gegenÃ¼berliegende Seite gefahren aber Korridor ist eng. Daher nach CW drehen - nicht das er wieder in Korridor fÃ¤hrt
            if(history[0].distanceDriven < 30  &&  bb.flagCoilFirstOutside == CO_LEFT) {
                randAngle = 50;
                bb.flagForceRotateDirection = FRD_CW;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 3 FRD_CW\n\r");
                }
            } else {
                //Do nothing
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 3 nothing\n\r");
                }
            }
            stateStuck = 0;

        } else if(  stateStuck == 4) { // //Im State 2 wurde CW gedreht und nun ist er auf die gegenÃ¼berliegende Seite gefahren aber Korridor ist eng. Daher nach CC drehen  - nicht das er wieder in Korridor fÃ¤hrt
            if(history[0].distanceDriven < 30  &&  bb.flagCoilFirstOutside == CO_RIGHT) {
                randAngle = 50;
                bb.flagForceRotateDirection = FRD_CC;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 4 FRD_CC\n\r");
                }
            } else {
                //Do nothing
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 4 nothing\n\r");
                }
            }
            stateStuck = 0;
        }



        /**********************
        * Drehung entprechend random Winkel starten
        **********************/

        // flagForceRotateDirection berÃ¼cksichtigen
        // Drehrichtung Ã¤nderen. Wenn bei rotation Ã¼ber perimeter gefahren wurde und beim befreien TPerRotateInsideCW, TPerRotateInsideCC beendet wurden, muss weiter
        // in dieselbe richtung wie  TPerRotateInsideCW, TPerRotateInsideCC gedreht haben, gedreht werden.

        if(bb.flagForceRotateDirection == FRD_CW) {
            errorHandler.setInfo("!05,Force Rotation to FRD_CW\r\n");
            bb.flagCoilFirstOutside = CO_LEFT;
            bb.flagForceRotateDirection = FRD_NONE;
        } else if(bb.flagForceRotateDirection == FRD_CC) {
            errorHandler.setInfo("!05,Force Rotation to FRD_CC\r\n");
            bb.flagCoilFirstOutside = CO_RIGHT;
            bb.flagForceRotateDirection = FRD_NONE;


            // When both coils are outside decide where to go on the result of the coil who was first outside last time.
        }
        if (bb.flagCoilFirstOutside == CO_BOTH) { // Beide Coils waren gleichzeitig drauÃŸen
            if ( history[0].lastRotDirection == DD_ROTATECW ) {
                bb.motor.turnTo(-1*randAngle,bb.cruiseSpeed);
                bb.driveDirection =  DD_ROTATECC;
            } else {
                bb.motor.turnTo(randAngle,bb.cruiseSpeed);
                bb.driveDirection =  DD_ROTATECW;
            }
        } else if ( bb.flagCoilFirstOutside == CO_LEFT) {
            bb.motor.turnTo(randAngle,bb.cruiseSpeed);
            bb.driveDirection =  DD_ROTATECW;
        } else { // if ( bb.flagCoilFirstOutside == CO_RIGHT) {
            bb.motor.turnTo(-1*randAngle,bb.cruiseSpeed);
            bb.driveDirection =  DD_ROTATECC;
        }


        shiftHistory(); // to insert values from the current rotation. When rotate is called again, the measured values will be inserted
        addAngleToHistory(randAngle);
        addDirectionToHistory(bb.driveDirection);
        printNewDistanceAndArray();
    }


    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("TRotateWorkx  too long in state\r\n");
        } else if (bb.motor.isPositionReached()) {
            bb.motor.startDistanceMeasurementWorkx();
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }



};

//############################################

class TRotate308: public Node    // Each task will be a class (derived from Node of course).
{
private:
    THistory history[HISTROY_BUFSIZE];

    long randAngle; // calculated angle wich should be rotated

    int stateStuck; // State variable for stucking algorithm
    int stateAngle; //Statevariabel fÃ¼r winkelberechnung
    int memStateAngle; // Memorize state


    bool isArcNotInitialised;
    int numberArcBig;
    int numberArcSmall;
    int dodgeArcCounterBig;
    int dodgeArcCounterSmall;

    int angleCounter;


public:

    bool showValuesOnConsole;

    TRotate308()  {

        // Erstmal so fÃ¼llen, das bedingungen unten nicht gleich erfÃ¼llt sind.
        for(int i=0; i< HISTROY_BUFSIZE; i++) {
            history[i].distanceDriven= 300;
            history[i].lastRotDirection = DD_ROTATECC;
            history[i].lastRotAngle = 88;
            history[i].coilFirstOutside = CO_BOTH;
            history[i].coilOutDifference = 100.0f;
            history[i].coilOutAngle = 90.0f;
        }
        stateStuck = 0;
        isArcNotInitialised=true;
        stateAngle = 0;
        angleCounter = 0;
    }

    // Shift array to right  to insert new distance at 0
    void shiftHistory() {
        for(int i=HISTROY_BUFSIZE-1; i > 0; i--) {
            history[i] = history[i-1];
        }
    }
    void addDistanceToHistory(long d) {
        history[0].distanceDriven= d;
    }
    void addDirectionToHistory(enuDriveDirection d) {
        history[0].lastRotDirection =  d;
    }
    void addAngleToHistory(long d) {
        history[0].lastRotAngle = d;
    }
    void addCoilFirstOutsideToHistory(enuFlagCoilsOutside c) {
        history[0].coilFirstOutside = c;
    }
    void addCoilOutDifferenceToHistory(float d) {
        history[0].coilOutDifference = d;
    }
    void addCoilOutAngleToHistory(float d) {
        history[0].coilOutAngle = d;
    }


    void printLastDistanceAndDirectionArray() {
        if(showValuesOnConsole == true) {

            sprintf(errorHandler.msg,"!05,distanceDriven: %ld %ld  %ld\r\n",history[0].distanceDriven,history[1].distanceDriven,history[2].distanceDriven);
            errorHandler.setInfo();

            sprintf(errorHandler.msg,"!05,coilFirstOutside %d %d %d\r\n",history[0].coilFirstOutside,history[1].coilFirstOutside,history[2].coilFirstOutside);
            errorHandler.setInfo();

            sprintf(errorHandler.msg,"!05,coilOutDifference: %f %f %f\r\n",history[0].coilOutDifference,history[1].coilOutDifference,history[2].coilOutDifference);
            errorHandler.setInfo();

            sprintf(errorHandler.msg,"!05,coilOutAngle %f %f %f\r\n",history[0].coilOutAngle,history[1].coilOutAngle,history[2].coilOutAngle);
            errorHandler.setInfo();

            sprintf(errorHandler.msg,"!05,lastRotDirection ");
            errorHandler.setInfo();
            for(int i = 0; i < HISTROY_BUFSIZE; i++) {
                if(history[i].lastRotDirection == DD_ROTATECW) {
                    sprintf(errorHandler.msg,"CW ");
                } else if (history[i].lastRotDirection == DD_ROTATECC) {
                    sprintf(errorHandler.msg,"CC ");
                } else {
                    sprintf(errorHandler.msg,"NA! ");
                }

                errorHandler.setInfo();
            }
            errorHandler.setInfo("\r\n");

            sprintf(errorHandler.msg,"!05,lastRotAngle: %ld %ld %ld\r\n",history[0].lastRotAngle, history[1].lastRotAngle, history[2].lastRotAngle);
            errorHandler.setInfo();
        }
    }


    void printNewDistanceAndArray() {

        if(showValuesOnConsole == true) {
            sprintf(errorHandler.msg,"!05,new rotDirection: ");
            errorHandler.setInfo();

            if(history[0].lastRotDirection == DD_ROTATECW) {
                sprintf(errorHandler.msg,"CW ");
            } else if (history[0].lastRotDirection == DD_ROTATECC) {
                sprintf(errorHandler.msg,"CC ");
            } else {
                sprintf(errorHandler.msg,"NA! ");
            }

            errorHandler.setInfo();
            errorHandler.setInfo("\r\n");


            sprintf(errorHandler.msg,"!05,new angle: %ld\r\n",history[0].lastRotAngle);
            errorHandler.setInfo();
        }
    }



    virtual void onInitialize(Blackboard& bb) {

        showValuesOnConsole = GETTB(TB_SHOW_ROTATE);

        // Insert data captured from last rotation to now
        addDistanceToHistory(bb.motor.getDistanceInCMForWorkx()); // Driven distance from end of last rotating until now
        addCoilFirstOutsideToHistory(bb.flagCoilFirstOutside);    // which coil was first outside jet
        addCoilOutDifferenceToHistory(bb.motor.getDistanceDiffInCMForCoilOut()); // Measured difference abs(coilL-coilR) of the way, the coils needed to go from outside to inside in cm
        addCoilOutAngleToHistory(bb.motor.getDistanceAngleCoilOut()); // Meassured Angle of coils to Perimeter 0Â° Means Both coils parrallel outside. 90Â° Robot is parrallel to perimeter (not really measureable)

        // Im Array steht nun an erster Position [0], die letzte gefahrene Distanz, der letzte gedrehte Winkel und die letzte gedrehte Richtung
        // Sowie die aktuellen Werte fÃ¼r: flagCoilFirstOutside, getDistanceDiffInCMForCoilOut, getDistanceAngleCoilOut
        // Auf Grundlage dieser Wert mÃ¼ssen nun Entscheidungen getroffen werden. Unten werden dann der neue Winkel und Winkelrichtung eingetragen, beim nÃ¤chsten Aufruf dieser
        // Funktion die dazugehÃ¶rige distanz in [0] oben eingetragen

        printLastDistanceAndDirectionArray() ;


        if(isArcNotInitialised) {
            numberArcBig = myRandom(15, 33);
            numberArcSmall  = myRandom(7, 23);
            dodgeArcCounterBig = 0;
            dodgeArcCounterSmall = 0;
            angleCounter = 0;
            DRRN(debug->printf("numberArcBig: %d;\r\n",numberArcBig);)
            DRRN(debug->printf("numberArcSmall: %d;\r\n",numberArcSmall);)
            isArcNotInitialised = false;
            memStateAngle = 0;
        }

        /**********************
        * Drehwinkel berechnen
        **********************/

        // Wenn strecke > 300cm dann standardwinkel wiederherstellen
        if(history[0].distanceDriven > 300&& stateAngle > 1) {
            stateAngle = memStateAngle;
            stateStuck = 0;
        }

        // Standarddrehwinkel festlegen
        if(stateAngle == 0 || stateAngle == 1) {  // 0 = goÃŸer Winkel, 1 = kleiner Winkel


            // Drehwinkel berechnen in AbhÃ¤ngigkeit von dodgeArcSmallCounter
            if(dodgeArcCounterBig  < numberArcBig  ) { // GroÃŸen Winkel rotieren
                dodgeArcCounterBig++;

                if(angleCounter <2) {
                    randAngle = myRandom(160,170);
                    errorHandler.setInfo("!05,>160-170\r\n");
                    angleCounter++;
                } else if(angleCounter <5) {
                    randAngle = myRandom(140, 160);
                    errorHandler.setInfo("!05,>140-160\r\n");
                    angleCounter++;
                } else if(angleCounter < 8) {
                    randAngle = myRandom(115, 140);
                    errorHandler.setInfo("!05,>115-140\r\n");
                    angleCounter++;
                } else if(angleCounter <10) {
                    randAngle = myRandom(105, 115);
                    errorHandler.setInfo("!05,>105-115\r\n");
                    angleCounter++;
                } else if(angleCounter <11) {
                    randAngle = 100;
                    errorHandler.setInfo("!05,>90\r\n");
                    angleCounter = 0;
                }

                /*
                 if(history[0].distanceDriven > 250 ) {
                     randAngle = myRandom(90, 155);  // Normaler Drehwinkel fÃ¼r groÃŸen Winkel
                     errorHandler.setInfo("!05,>250/0 Area 90-155\r\n");
                 } else {  //Wenn weniger als 250m gefahren wurde kleineren Winkel wÃ¤hlen
                     if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                         randAngle = myRandom(70, 100); // Wenn beide Spulen drauÃŸen sind, kann Drehwinkel grÃ¶ÃŸer sein 80 155
                         errorHandler.setInfo("!05,<250/0 Area 70-100\r\n");
                     } else {
                         randAngle = myRandom(50, 70); // Wenn eine Spule drauÃŸenist, nicht soweit drehen
                         errorHandler.setInfo("!05,<250/0 Area 50-70\r\n");
                     }
                 }*/

                stateAngle = 0;

            } else { // Kleinen Winkel rotieren
                //randAngle = myRandom(50, 60);   // Normaler Drehwinkel fÃ¼r kleinen Winkel 50/60
                //errorHandler.setInfo("!05,250Area 50-60\r\n");

                if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                    randAngle = myRandom(50, 70);   // Welchen Winkel soll robbi drehen wenn zu oft ein kurzer weg gefahrebn wurde. 50/90
                    errorHandler.setInfo("!05,<50-70\r\n");
                } else {
                    randAngle = myRandom(30, 50);   // Wenn eine Spule drauÃŸen ist, nicht soweit drehen 40/60
                    errorHandler.setInfo("!05,<30-50\r\n");
                }

                dodgeArcCounterSmall++;
                stateAngle = 1;
            }

            // dodgeArcSmallCounter reseten und wieder von vorne mit groÃŸem winkel beginnen
            if (dodgeArcCounterSmall  >= numberArcSmall  ) {
                isArcNotInitialised = true;
            }
        }

        /// ####Vermutlich Ecke erwischt. Dann weiter in gleiche Richtung drehen.                                                             // Nur aktivieren wenn wenn in state 0 oder 1
        if(history[0].distanceDriven < 50 && history[1].distanceDriven > 150 &&  history[0].coilFirstOutside != history[1].coilFirstOutside && stateAngle<2) {

            if(history[0].lastRotDirection== DD_ROTATECC) {
                // Weiter CC drehen wenn vorher CC gedreht wurde
                bb.flagForceRotateDirection = FRD_CC;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,Ecke lastRotDirection[0]== DD_ROTATECC\n\r");
                }
            } else {
                // Weiter CW drehen wenn vorher CW gedreht wurde
                bb.flagForceRotateDirection = FRD_CW;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,Ecke lastRotDirection[0]== DD_ROTATECW\n\r");
                }
            }


            if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                randAngle = myRandom(70, 100); // Wenn beide Spulen drauÃŸen sind, kann Drehwinkel grÃ¶ÃŸer sein 80 155
            } else {
                randAngle = myRandom(50, 70); // Wenn eine Spule drauÃŸenist, nicht soweit drehen
            }
            errorHandler.setInfo("!05,Ecke\r\n");
        }

        // Eintrittsbedingung fÃ¼r Korridor                                   Only if big arc      latch function
        if( (history[0].distanceDriven < 150 && history[1].distanceDriven < 150  && stateAngle == 0 )  || stateAngle == 10 ) { // history[0].coilFirstOutside != history[1].coilFirstOutside &&

            if( stateAngle == 0 || stateAngle ==1) {
                memStateAngle = stateAngle;
            }
            stateAngle = 10;

            // Im Korridor nicht so stark drehen
            if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                randAngle = myRandom(50, 90);   // Welchen Winkel soll robbi drehen wenn zu oft ein kurzer weg gefahrebn wurde. 50/90
            } else {
                randAngle = myRandom(40, 60); // Wenn eine Spule drauÃŸen ist, nicht soweit drehen 40/60
            }
            errorHandler.setInfo("!05,Korridor\n\r");
        }


        // Test auf  Korridor
        // Eintrittsbedingung fÃ¼r schmalen Korridor aber nur wenn ober groÃŸer Winkel gefahren wird.
        if( (history[0].distanceDriven < 80 && history[1].distanceDriven < 80  && stateAngle == 0 )) { // history[0].coilFirstOutside != history[1].coilFirstOutside
            if( stateAngle == 0 || stateAngle ==1) {
                memStateAngle = stateAngle;
            }
            stateAngle = 30;
            errorHandler.setInfo("!05,Schmaler Korridor 1\n\r");
        }


        if( (history[0].distanceDriven < 80 && history[1].distanceDriven < 80  && history[2].distanceDriven < 80) || stateAngle == 30 ) {

            if( stateAngle == 0 || stateAngle ==1) {
                memStateAngle = stateAngle;
            }
            stateAngle = 30; // latch

            // Im schmalem Korridor nicht so stark drehen
            if(bb.flagCoilOutsideAfterOverrun  == CO_BOTH) {
                randAngle = myRandom(50, 70);   // Welchen Winkel soll robbi drehen wenn zu oft ein kurzer weg gefahrebn wurde. 50/90
            } else {
                randAngle = myRandom(40, 50);   // Wenn eine Spule drauÃŸen ist, nicht soweit drehen 40/60
            }
            errorHandler.setInfo("!05,Schmaler Korridor 2\n\r");
        }


        if(bb.flagForceSmallRotAngle > 0) {
            randAngle = myRandom(30, 60);
            bb.flagForceSmallRotAngle--;
            errorHandler.setInfo("!05,ForceSmallRotAngle\r\n");
        }

        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;



        /**********************
        * Oszilliert Roboter an Perimeter?
        * Overrides the randAngle from above if Robot stucks
        **********************/
        // Wenn eine Strecke weniger als 20 cm gefahren wurde, dann wird angenommen, dass Ende Korridor Erreicht ist und es muss gedreht werden
        if(  (history[0].distanceDriven <20) && (stateStuck == 0) && (bb.flagCoilOutsideAfterOverrun == CO_BOTH) ) {

            if(showValuesOnConsole) {
                errorHandler.setInfo("!05,STUCK distanceDriven[0] <20\r\n");
            }
            //randAngle = 160-lastAngle[0] ;

            randAngle = 90;

            if(history[0].lastRotDirection== DD_ROTATECC) {
                // Weiter CC drehen wenn vorher CC gedreht wurde
                bb.flagForceRotateDirection = FRD_CC;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 0 lastRotDirection[0]== DD_ROTATECC\n\r");
                }
                stateStuck = 1;
            } else {
                // Weiter CW drehen wenn vorher CW gedreht wurde
                bb.flagForceRotateDirection = FRD_CW;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 0 lastRotDirection[0]== DD_ROTATECW\n\r");
                }
                stateStuck = 2;
            }
        } else if(  stateStuck == 1) { //CC gedreht
            if(history[0].distanceDriven > 40  || bb.flagCoilOutsideAfterOverrun  != CO_BOTH) {
                //Do nothing
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 1 nothing\n\r");
                }
                stateStuck = 0;
            } else {
                // Weiter CC drehen
                randAngle = 90;
                bb.flagForceRotateDirection = FRD_CC;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 1 FRD_CC\n\r");
                }
                stateStuck = 3;
            }


        } else if(  stateStuck == 2) { //CW gedreht
            if(history[0].distanceDriven > 40  || bb.flagCoilOutsideAfterOverrun  != CO_BOTH) {
                //Do nothing
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 2 nothing\n\r");
                }
                stateStuck = 0;
            } else {
                // Weiter CW drehen
                randAngle = 90;
                bb.flagForceRotateDirection = FRD_CW;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 2 FRD_CW\n\r");
                }
                stateStuck = 4;
            }



        } else if(  stateStuck == 3) { //Im State 1 wurde CC gedreht und nun ist er auf die gegenÃ¼berliegende Seite gefahren aber Korridor ist eng. Daher nach CW drehen - nicht das er wieder in Korridor fÃ¤hrt
            if(history[0].distanceDriven < 30  &&  bb.flagCoilFirstOutside == CO_LEFT) {
                randAngle = 50;
                bb.flagForceRotateDirection = FRD_CW;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 3 FRD_CW\n\r");
                }
            } else {
                //Do nothing
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 3 nothing\n\r");
                }
            }
            stateStuck = 0;

        } else if(  stateStuck == 4) { // //Im State 2 wurde CW gedreht und nun ist er auf die gegenÃ¼berliegende Seite gefahren aber Korridor ist eng. Daher nach CC drehen  - nicht das er wieder in Korridor fÃ¤hrt
            if(history[0].distanceDriven < 30  &&  bb.flagCoilFirstOutside == CO_RIGHT) {
                randAngle = 50;
                bb.flagForceRotateDirection = FRD_CC;
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 4 FRD_CC\n\r");
                }
            } else {
                //Do nothing
                if(showValuesOnConsole) {
                    errorHandler.setInfo("!05,stateStuck 4 nothing\n\r");
                }
            }
            stateStuck = 0;
        }



        /**********************
        * Drehung entprechend random Winkel starten
        **********************/

        // flagForceRotateDirection berÃ¼cksichtigen
        // Drehrichtung Ã¤nderen. Wenn bei rotation Ã¼ber perimeter gefahren wurde und beim befreien TPerRotateInsideCW, TPerRotateInsideCC beendet wurden, muss weiter
        // in dieselbe richtung wie  TPerRotateInsideCW, TPerRotateInsideCC gedreht haben, gedreht werden.

        if(bb.flagForceRotateDirection == FRD_CW) {
            errorHandler.setInfo("!05,Force Rotation to FRD_CW\r\n");
            bb.flagCoilFirstOutside = CO_LEFT;
            bb.flagForceRotateDirection = FRD_NONE;
        } else if(bb.flagForceRotateDirection == FRD_CC) {
            errorHandler.setInfo("!05,Force Rotation to FRD_CC\r\n");
            bb.flagCoilFirstOutside = CO_RIGHT;
            bb.flagForceRotateDirection = FRD_NONE;


            // When both coils are outside decide where to go on the result of the coil who was first outside last time.
        }
        if (bb.flagCoilFirstOutside == CO_BOTH) { // Beide Coils waren gleichzeitig drauÃŸen
            if ( history[0].lastRotDirection == DD_ROTATECW ) {
                bb.motor.turnTo(-1*randAngle,bb.cruiseSpeed);
                bb.driveDirection =  DD_ROTATECC;
            } else {
                bb.motor.turnTo(randAngle,bb.cruiseSpeed);
                bb.driveDirection =  DD_ROTATECW;
            }
        } else if ( bb.flagCoilFirstOutside == CO_LEFT) {
            bb.motor.turnTo(randAngle,bb.cruiseSpeed);
            bb.driveDirection =  DD_ROTATECW;
        } else { // if ( bb.flagCoilFirstOutside == CO_RIGHT) {
            bb.motor.turnTo(-1*randAngle,bb.cruiseSpeed);
            bb.driveDirection =  DD_ROTATECC;
        }


        shiftHistory(); // to insert values from the current rotation. When rotate is called again, the measured values will be inserted
        addAngleToHistory(randAngle);
        addDirectionToHistory(bb.driveDirection);
        printNewDistanceAndArray();
    }


    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("TRotateWorkx  too long in state\r\n");
        } else if (bb.motor.isPositionReached()) {
            bb.motor.startDistanceMeasurementWorkx();
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }



};

/*
class TRotate308: public Node    // Each task will be a class (derived from Node of course).
{
public:
    bool isRotateNotInitialised;
    int numberRotateRigth;
    int numberRotateLeft;
    int dodgeDirectionCounterLeft;
    int dodgeDirectionCounterRight;

    bool isArcNotInitialised;
    int numberArcBig;
    int numberArcSmall;
    int dodgeArcCounterBig;
    int dodgeArcCounterSmall;




    TRotate308():isRotateNotInitialised(true),
        isArcNotInitialised(true)
    {}

    virtual void onInitialize(Blackboard& bb) {

        int randNumber;


//for(int i = 1; i<100; i++){
        if(isArcNotInitialised) {
            numberArcBig = myRandom(7, 23);
            numberArcSmall  = myRandom(7, 23);
            dodgeArcCounterBig = 0;
            dodgeArcCounterSmall = 0;

            DRRN(debug->printf("numberArcBig: %d;\r\n",numberArcBig);)
            DRRN(debug->printf("numberArcSmall: %d;\r\n",numberArcSmall);)
            isArcNotInitialised = false;
        }

        if(isRotateNotInitialised || bb.flagForceRotateDirection != FRD_NONE) { // Wenn direction vorgegeben wird, auch neu initialisieren
            numberRotateRigth = myRandom(7, 23);
            numberRotateLeft  = myRandom(7, 23);
            dodgeDirectionCounterRight = 0;
            dodgeDirectionCounterLeft = 0;

            DRRN(debug->printf("numberRotateRigth: %d;\r\n",numberRotateRigth);)
            DRRN(debug->printf("numberRotateLeft: %d;\r\n",numberRotateLeft);)
            isRotateNotInitialised = false;
        }

//isArcNotInitialised = true;
//isRotateNotInitialised  = true;
//}
        // Drehwinkel berechnen in AbhÃ¤ngigkeit von dodgeArcSmallCounter
        if(dodgeArcCounterBig  < numberArcBig  ) { // GroÃŸen Winkel rotieren
            randNumber = myRandom(90, 155);  // Normaler Drehwinkel fÃ¼r groÃŸen Winkel
            dodgeArcCounterBig++;
        } else { // Kleinen Winkel rotieren wenn dodgeArcSmallCounter>= 15 ist
            randNumber = myRandom(50, 60);   // Normaler Drehwinkel fÃ¼r kleinen Winkel
            dodgeArcCounterSmall++;
        }

        // dodgeArcSmallCounter reseten und wieder von vorne mit groÃŸem winkel beginnen
        if (dodgeArcCounterSmall  >= numberArcSmall  ) {
            isArcNotInitialised = true;
        }


        if(bb.flagForceSmallRotAngle > 0) {
            randNumber = myRandom(50, 60);
            bb.flagForceSmallRotAngle--;
        }

        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;

        // Drehrichtung Ã¤nderen. Wenn bei rotation Ã¼ber perimeter gefahren wurde und beim befreien TPerRotateInsideCW, TPerRotateInsideCC beendet wurden, muss weiter
        // in dieselbe richtung wie  TPerRotateInsideCW, TPerRotateInsideCC gedreht haben, gedreht werden.
        switch(bb.flagForceRotateDirection) {
            case FRD_CW  :
                errorHandler.setInfo("!03,Change Rotation to FRD_CW;\r\n");
                // Rotate CW und CC wurde oben neu initialisieren da nun CW drehen soll
                bb.flagForceRotateDirection = FRD_NONE;
                break;
            case FRD_CC  :
                errorHandler.setInfo("!03,Change Rotation to FRD_CC ;\r\n");
                // Rotate CW und CC wurde oben neu initialisiert. Es soll hier aber nur CC gedreht werden, daher werte so setzen, dass in unterer if bedingung CW nicht ausgefÃ¼hrt wird.
                numberRotateRigth = 99;
                dodgeDirectionCounterRight = 999;
                bb.flagForceRotateDirection = FRD_NONE;
                break;
            default : //Optional
                break;
        }// Switch



        // Drehrichtung bestimmen in abhÃ¤ngigkeit von dodgeDirectionCounter
        if (dodgeDirectionCounterRight < numberRotateRigth) { // AnfÃ¤nglich rechts drehen
            bb.motor.turnTo(randNumber,bb.cruiseSpeed);
            bb.driveDirection =  DD_ROTATECW;
            dodgeDirectionCounterRight++;
        } else { //Wenn dodgeDirectionCounterRight >=numberRotateRigth  ist links rotieren
            bb.motor.turnTo(-1*randNumber,bb.cruiseSpeed);
            bb.driveDirection =  DD_ROTATECC;
            dodgeDirectionCounterLeft++;
        }

        // dodgeDirectionCounter reseten und wieder von vorne mit rechtsdrehen beginnen
        if (dodgeDirectionCounterLeft >= numberRotateLeft) {
            isRotateNotInitialised = true;
        }

    } //onInitialize

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,TRotate308  too long in state\r\n");
        }

        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }
        return BH_RUNNING;
    }

};

*/










#endif

