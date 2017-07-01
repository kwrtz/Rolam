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

#ifndef CHARGESYSTEM_H
#define CHARGESYSTEM_H

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

class TchargeSystem : public Thread
{
private:
    uint8_t count;

public:

    bool flagShowChargeSystem;

    float sensorValueCV;
    float sensorValueCC;
    float chargeVoltage;
    float chargeCurrent;

    void setup() {
        sensorValueCV = aiCHARGEVOLTAGE.read(); // Converts and read the analog input value (value from 0.0 to 1.0)
        sensorValueCC = aiCHARGECURRENT.read();
        chargeVoltage = 0;
        chargeCurrent = 0;
        flagShowChargeSystem = false;
        count = 20;
    }

    void activateRelay() {
        if(doChargeEnable ==  0)
            doChargeEnable = 1;
    }

    void deactivateRelay() {
        if(doChargeEnable !=  0)
            doChargeEnable = 0;
    }


    virtual void run() {
        // Wird alle 53ms aufgerufen
        runned();

        sensorValueCV = aiCHARGEVOLTAGE.read(); // Converts and read the analog input value (value from 0.0 to 1.0)
        float chargeVoltage1 = sensorValueCV * 34.0196248f;
        sensorValueCC = aiCHARGECURRENT.read();
        float chargeCurrent1 = sensorValueCC * 1.42f; 

        const float accel = 0.1f;

        if (abs(chargeVoltage - chargeVoltage1)>5)
            chargeVoltage = chargeVoltage1;
        else
            chargeVoltage = (1.0f - accel) * chargeVoltage + accel * chargeVoltage1;

        if (abs(chargeCurrent - chargeCurrent1)>5)
            chargeCurrent = chargeCurrent1;
        else
            chargeCurrent = (1.0f - accel) * chargeCurrent + accel * chargeCurrent1;


        if(flagShowChargeSystem && (++count > 20) ) {

            if(isBatFull()) {
                sprintf(errorHandler.msg,"!03,FULL adcCV: %f CV: %f  adcCC: %f  CC: %f Watt: %f\r\n", sensorValueCV, chargeVoltage, sensorValueCC, chargeCurrent, chargeVoltage*chargeCurrent);
                errorHandler.setInfo(); 
            } else {
                sprintf(errorHandler.msg,"!03,CHARGING adcCV: %f CV: %f  adcCC: %f  CC: %f Watt: %f\r\n", sensorValueCV, chargeVoltage, sensorValueCC, chargeCurrent, chargeVoltage*chargeCurrent);
                errorHandler.setInfo(); 

            }
            count = 0;
        }

    }

    bool isInChargingStation() {
        if (chargeVoltage > 15.0f)  {
            return true;
        }

        return false;
    }


    bool isBatFull() {
        float watt = chargeVoltage * chargeCurrent;
        if (watt < 8.0f)  {
            return true;
        }

        return false;
    }


};

#endif
