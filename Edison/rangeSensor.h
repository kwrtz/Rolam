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

#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Thread.h"
#include "global.h"
#include "hardware.h"
#include "errorhandler.h"


class TrangeSensor : public Thread
{
private:

public:
    int range1;
    //int light1;

    bool flagShowRange;

    void setup() {
/*
        range1 = 0;
        //light1 = 0;
        flagShowRange = false;

        rangeMod1.setRangeRegister(30); //1,3m
        wait(0.01);
        rangeMod1.setMaxGainRegister(25);
        wait(0.01);
        rangeMod1.startRanging();
		*/
    }

    virtual void run() {
        // Wird alle 87ms aufgerufen
        runned();
/*
        if(rangeMod1.rangingFinished()) {
            range1 = rangeMod1.getRange();
            //light1 = rangeMod1.getLightIntensity();
            if(flagShowRange) {
                sprintf(errorHandler.msg,"!03, Range_1: %i\r\n", range1);
                errorHandler.setInfo();
            }
            rangeMod1.startRanging();
        }
		*/
    }

    bool isNearObstacle() {
		return false;
		//return (range1 < 85 && range1 > 0); //0 bedeutet kein Signal empfangen
    }
};

#endif
