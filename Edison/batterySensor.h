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

#ifndef BATTERIESENSOR_H
#define BATTERIESENSOR_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Thread.h"
#include "global.h"
#include "hardware.h"
#include "errorhandler.h"

extern TErrorHandler errorHandler;

class TbatterieSensor : public Thread
{
private:

public:

    float sensorValue;
    float voltage;

    void setup() {
        sensorValue = aiBATVOLT.read(); // Converts and read the analog input value (value from 0.0 to 1.0)
        voltage = 29;
    }


    virtual void run() {
        // Wird alle 1000ms aufgerufen
        runned();

        sensorValue = aiBATVOLT.read(); // Converts and read the analog input value (value from 0.0 to 1.0)
        float batvolt = sensorValue * 34.0196248f;

        const float accel = 0.1;

        if (abs(voltage - batvolt)>5)
            voltage = batvolt;
        else
            voltage = (1.0f - accel) * voltage + accel * batvolt;

        if (voltage < 21.7f) {
            sprintf(errorHandler.msg,"!03,arbitrator switch off voltage reached\r\n");
            errorHandler.setError(); 
        }
    }

    bool isVoltageLow() {
        if (voltage < 23.7f) {
            return true;
        }

       return false;
    }

};

#endif
