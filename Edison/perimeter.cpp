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

//#include <algorithm>   //for min/max
#include "perimeter.h"
#include "helpers.h"
#include "hardware.h"
#include "errorhandler.h"



void TPerimeterThread::setup()
{
    checksumError = 0;

    signalCounterL = 0;    // -2 inside >=  wert  <=+2 outside
    magnetudeL = 0;       // maximum des korrelationssignals. Kann auch negative sein. Amplitude von kleiner +-40 ist rauschen
    lastTimeSignalReceivedL = 0;

    signalCounterR = 0;
    magnetudeR = 0;
    lastTimeSignalReceivedR = 0;


    signalCounterB = 0;
    magnetudeB = 0;
    lastTimeSignalReceivedB = 0;

    perimeterLastTransitionTimeB = 0;

    magMax = 0;
    magMaxB = 0;
    showValuesOnConsole = false;
    arcToPerimeter = 0;

    //magLineFollowMax =  32100;
    //magLineFollow = 0;
    //magLineFollowMin = -32100;


    count = 0;

    SetState(SPR_HEADER1);
}


void TPerimeterThread::CaluculateInsideOutside( int16_t magl, int16_t magr, int16_t magb)
{

    magnetudeL = magl;
    magnetudeR = magr;
    magnetudeB = magb;


    //----------------------------------------
    // Auswerten Links
    //----------------------------------------

    // ** inside mag positive**
    if ( magl > 0) {
        signalCounterL = min(signalCounterL + 1, 2);

        if (signalCounterL == 0) // 0 Ã¼berspringen
            signalCounterL++;


        lastTimeSignalReceivedL = millis();

        // Determine Maximum amplitude
        if (magl > magMax) {
            magMax = (magMax * 9 + magl * 1) / 10;
        }
    }

    // ** outside mag negative**
    else if (magl < 0) {
        signalCounterL = max(signalCounterL - 1, -2);

        if (signalCounterL == 0) // 0 Ã¼berspringen
            signalCounterL--;

        lastTimeSignalReceivedL = millis();
    }

    // ** lost ** _magnetudeL.sIn16t == 0
    else {  // Wenn kein Signal erkannt wurde ist amplitude 0. Dann wird das letzte Signal als aktuell angenommen bzw. eingefroren.
        // Wenn Sensor direkt Ã¼ber der Leitung ist Amplitude auch 0
        // Ãœber signalTimedOut kann dann festgestellt werden, ob das Signal zu lange nicht mehr auftrat
        /*
        if (signalCounterL > 0)
            signalCounterL--;
        if (signalCounterL < 0)
            signalCounterL++;
        */
    }



    //----------------------------------------
    // Auswerten Rechts
    //----------------------------------------

    if (magr > 0) {
        signalCounterR = min(signalCounterR + 1, 2);  //Inside

        if (signalCounterR == 0) // 0 Ã¼berspringen
            signalCounterR++;

        lastTimeSignalReceivedR = millis();

        // Determine Maximum amplitude
        if (magr > magMax) {
            magMax = (magMax * 9 + magr * 1) / 10;
        }


    } else if (magr < 0) {
        signalCounterR = max(signalCounterR - 1, -2); //Outside

        if (signalCounterR == 0)
            signalCounterR--;

        lastTimeSignalReceivedR = millis();

    } else { // Wenn kein Signal erkannt wurde ist amplitude 0. Dann wird das letzte Signal als aktuell angenommen bzw. eingefroren.
        // Wenn Sensor direkt Ã¼ber der Leitung ist Amplitude auch 0
        // Ãœber signalTimedOut kann dann festgestellt werden, ob das Signal zu lange nicht mehr auftrat
        /*
        if (signalCounterR > 0)
           signalCounterR--;
        if (signalCounterR < 0)
           signalCounterR++;
        */
    }



    //----------------------------------------
    // Auswerten Hinten
    //----------------------------------------

    if (magb > 0) {
        signalCounterB = min(signalCounterB + 1, 2);  //inside

        if (signalCounterB == 0) { // 0 Ã¼berspringen
            signalCounterB++;
            perimeterLastTransitionTimeB = millis();
        }

        lastTimeSignalReceivedB = millis();

        // Determine Maximum amplitude
        if (magb > magMaxB) {
            magMaxB = (magMaxB * 9 + magb * 1) / 10;
        }

    } else if (magb < 0) {
        signalCounterB = max(signalCounterB - 1, -2); // outside

        if (signalCounterB == 0) { // 0 Ã¼berspringen
            signalCounterB--;
            perimeterLastTransitionTimeB = millis();
        }

        lastTimeSignalReceivedB = millis();

    } else { // Wenn kein Signal erkannt wurde ist amplitude 0. Dann wird das letzte Signal als aktuell angenommen bzw. eingefroren.
        // Wenn Sensor direkt Ã¼ber der Leitung ist Amplitude auch 0
        // Ãœber signalTimedOut kann dann festgestellt werden, ob das Signal zu lange nicht mehr auftrat


        /*
        if (signalCounterB > 0)
           signalCounterB--;
        if (signalCounterB < 0)
           signalCounterB++;
        */
    }


    if (showValuesOnConsole) {
        sprintf(errorHandler.msg,"!03,ML: %d MR: %d MB: %d CL:%d CR: %d CB: %d\r\n",magl , magr, magb , signalCounterL ,signalCounterR, signalCounterB );
        errorHandler.setInfo();
    }


}


void TPerimeterThread::CaluculateArcToPerimeter( )
{
    float normalisedAmplitudeL,normalisedAmplitudeR, arc;
    float magMax = 12000;


    if( magMax == 0 || magnetudeR == 0 || magnetudeL == 0)
        return;

    if( isLeftOutside() || isRightOutside())
        return;


    normalisedAmplitudeL = magnetudeL/magMax;
    normalisedAmplitudeR = magnetudeR/magMax;

    /*
        //Make sure the working variables are within the new limits.
        if(normalisedAmplitudeL >1.0f){
            normalisedAmplitudeL = 1.0f;
        } else if (normalisedAmplitudeL < 0) {
            normalisedAmplitudeL = 0;
        }

        if(normalisedAmplitudeR >1.0f){
            normalisedAmplitudeR = 1.0f;
        } else if (normalisedAmplitudeR < 0) {
            normalisedAmplitudeR = 0;
        }
      */
    arc= (normalisedAmplitudeL - normalisedAmplitudeR);

    count++;
    if(count>10) {
        sprintf(errorHandler.msg,"!03,L: %d R:%d arc: %f\r\n",magnetudeL,magnetudeR,arc);
        errorHandler.setInfo();
        count = 0;
    }



}
// Serielle Leitung einlesen
void TPerimeterThread::UpdateState( EPerReceiveState t )
{

    switch (t) {
        case SPR_HEADER1:
            header1 = perRX.getChar();
            if (header1 == 255) { // is this the header1
                SetState(SPR_HEADER2);
            } else {
                if (showValuesOnConsole) {
                    sprintf(errorHandler.msg,"!03,header1 not found: %hu\r\n",header1);
                    errorHandler.setInfo();
                }
            }
            break;

        case SPR_HEADER2:
            header2 = perRX.getChar();
            if (header2 == 255) { // is this the header2
                SetState(SPR_MAGNETUDEL);
            } else {
                SetState(SPR_HEADER1);
                if (showValuesOnConsole) {
                    sprintf(errorHandler.msg,"!03,header2 wrong: %hu\r\n",header2);
                    errorHandler.setInfo();
                }

            }
            break;

        case SPR_MAGNETUDEL:
            _magnetudeL.uBytes[0] = perRX.getChar();
            SetState(SPR_MAGNETUDEL1);
            break;

        case SPR_MAGNETUDEL1:
            _magnetudeL.uBytes[1] = perRX.getChar();
            SetState(SPR_MAGNETUDER);
            break;


        case SPR_MAGNETUDER:
            _magnetudeR.uBytes[0] = perRX.getChar();
            SetState(SPR_MAGNETUDER1);
            break;

        case SPR_MAGNETUDER1:
            _magnetudeR.uBytes[1] = perRX.getChar();
            SetState(SPR_MAGNETUDEB);
            break;

        case SPR_MAGNETUDEB:
            _magnetudeB.uBytes[0] = perRX.getChar();
            SetState(SPR_MAGNETUDEB1);
            break;

        case SPR_MAGNETUDEB1:
            _magnetudeB.uBytes[1] = perRX.getChar();
            SetState(SPR_CHECKSUM);
            break;


        case SPR_CHECKSUM:
            SetState(SPR_HEADER1);
            checksumm = perRX.getChar();

            checksummcalculated = header1 + header2;
            checksummcalculated += _magnetudeL.uBytes[0] + _magnetudeL.uBytes[1];
            checksummcalculated += _magnetudeR.uBytes[0] + _magnetudeR.uBytes[1];
            checksummcalculated += _magnetudeB.uBytes[0] + _magnetudeB.uBytes[1];
            checksummcalculated &= 0x7F; //B01111111;

            if (checksummcalculated == checksumm) {
                checksumError = 0;   //Kein Error
                //Serial.println("Checksum OK: ");

                // Von negative nach positive und andersherum wandeln. Inside = positive.
                CaluculateInsideOutside(-1*_magnetudeL.sIn16t , -1*_magnetudeR.sIn16t , -1*_magnetudeB.sIn16t);
                //CaluculateArcToPerimeter();
            } else {
                if (showValuesOnConsole) {
                    errorHandler.setInfo("!03,checksum wrong\rn");
                    sprintf(errorHandler.msg,"!03,%hu %hu %i %i %hu %hu\r\n",header1, header2, _magnetudeL.sIn16t , _magnetudeR.sIn16t ,  checksumm,  checksummcalculated);
                    errorHandler.setInfo();
                }
                checksumError++;    //Errorcounter hochzÃ¤hlen
                //neueDatenEmpfangen = false;  // Mitteilen, dass keine neuen Daten da sind
                //Serial.println("Checksum Not OK: ");
            }
            break;

        default:
            //TODO invalid state - reset, perhaps?
            break;
    }
}

void TPerimeterThread::run()
{
    runned();
    /*
    if(perRX->readable()){
      debug->printf("%i\r\n",perRX.getChar());
    }
    return;
    */
    //debug->printf("%i\r\n",perRX->readable() );
    while (perRX.readable() ) { //Solange buffer durchlaufen bis alle zeichen gelesen wurden
        LoopFSM();
    }
}

// ------------------------------------------------------------------------------------------
// Folgende Funktionen werden von bMow aufgerufen um Max des Perimeters zu bestimmen
// ------------------------------------------------------------------------------------------
bool TPerimeterThread::isNearPerimeter()
{

    if( magMax == 0) { //f we have not measured a magnitude always return near in order to drive low speed then
        return true;
    }

    //return false;
    long threshold = (magMax * 95L) / 100L; //95% vom Maximalwert ist untere schwelle fÃ¼r bestimmung ob nah am perimeter wire

    if (magnetudeL > threshold && magnetudeR > threshold)
        return true;

    if (magnetudeL > threshold && magnetudeR <=0)
        return true;

    if (magnetudeR > threshold && magnetudeL <= 0)
        return true;


    return false;
}


// ------------------------------------------------------------------------------------------



bool TPerimeterThread::isLeftInside()
{
    if (signalCounterL > 0)
        return true;
    return false;

}


bool TPerimeterThread::isRightInside()
{
    if (signalCounterR > 0)
        return true;
    return false;
}


bool TPerimeterThread::isBackInside()
{
    //if (signalCounterB > 0)
    //    return true;

    if (magnetudeB > 0) {
        return true;
    }

    return false;
}

bool TPerimeterThread::isLeftOutside()
{
    if (signalCounterL < 0)
        return true;

    return false;

}

bool TPerimeterThread::isRightOutside()
{
    if (signalCounterR < 0)
        return true;
    return false;
}

bool TPerimeterThread::isBackOutside()
{
    //if (signalCounterB < 0)
    //    return true;

    if (magnetudeB < 0)
        return true;


    return false;
}



