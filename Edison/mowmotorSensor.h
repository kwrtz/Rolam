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

#ifndef MOWMOROTRSENSOR_H
#define MOWMOROTRSENSOR_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Thread.h"
#include "global.h"
#include "hardware.h"
#include "batterySensor.h"
#include "errorhandler.h"
#include "RunningMedian.h"

extern TbatterieSensor batterieSensor;
extern TErrorHandler errorHandler;
extern TMowClosedLoopControlThread clcM;

// Liest MowMotorstrom aus und berechnet die Wattzahl
// Wenn diese zu hoch ist, wird der Mähmotor sofort ausgeschaltet.

class TMowMotorSensor : public Thread
{
private:
    int motorMowPowerMax;
    int motorMowSenseCounter ;
    unsigned long lastTimeMotorMowStucked;
    //unsigned long timeUnderHeavyLoad;
    uint8_t count;
    
public:

    bool showValuesOnConsole;
    bool motorUnderHeavyLoad;

    float sensorValue;
    float current;
    float watt;
    float offset;

    void setup() {
        count = 10;
        current = 0;
        motorMowSenseCounter = 0;
        motorMowPowerMax = 75;
        lastTimeMotorMowStucked = 0;
        showValuesOnConsole = false;
        motorUnderHeavyLoad = false;
        //timeUnderHeavyLoad = 0;
       measureOffset(); 
       sensorValue = aiMOWMOTCURRENT.read() - offset; // Converts and read the analog input value (value from 0.0 to 1.0)

      
    }


    virtual void run() {
        // Wird alle 198ms aufgerufen
        runned();

        sensorValue = aiMOWMOTCURRENT.read(); // Converts and read the analog input value (value from 0.0 to 1.0)
        sensorValue = fabs(sensorValue - offset);
        float sensorCurrent = sensorValue * 60.0f;

        //current = sensorCurrent;

        const float accel = 0.1f;
        current = (1.0f - accel) * current + accel * sensorCurrent;

        watt = batterieSensor.voltage * current;

        checkMowCurrent();
        checkIfUnderHeavyLoad();

        if(showValuesOnConsole && (++count > 10) ) {
			errorHandler.setInfo(F("!03,Watt: %f MotorCurrent: %f sensorValue: %f motorDisabled %d\r\n"),watt, current, sensorValue, clcM.motorDisabled);
            count = 0;
        }

    }


    void checkMowCurrent() {

        if (watt >= motorMowPowerMax) {
            motorMowSenseCounter++;
            if(motorMowSenseCounter >35){ // Überlauf verhindern
                motorMowSenseCounter = 35;
            }    
        } else {
            motorMowSenseCounter = 0;
            if (millis() >= lastTimeMotorMowStucked + 15000) { // wait 30 seconds before switching on again
                clcM.motorDisabled = false;
                //debug->printf( "Mow MotorDisabled\r\n");
            }
        }

        if (motorMowSenseCounter >= 30) { //ignore motorMowPower for 3 seconds
			errorHandler.setInfo(F("!03,Mow MotorDisabled: current high\r\n"));
            clcM.motorDisabled = true;
            lastTimeMotorMowStucked = millis();
        }
    }


    void checkIfUnderHeavyLoad () {
        // Mit Hysterese schalten
        if (watt >= 45) {
            if(motorUnderHeavyLoad==false) {
                motorUnderHeavyLoad = true;
                //timeUnderHeavyLoad = millis();
                //errorHandler.setInfo ("!03,Mowmotor under heavy load");
            }
        } else {
            if(motorUnderHeavyLoad==true) { // && (millis()-timeUnderHeavyLoad) > 2000 ) {
                if( watt < 30) {
                    motorUnderHeavyLoad = false;
                }
            }
        }
    }

    bool checkIfUnderLoad () {
        if (watt >= 30) {
            return true;
        } 
        else {
            return false;   
        }    
    }

    void measureOffset () {
        RunningMedian<float,16> myMedian;
        
        for (unsigned int i=0; i < myMedian.getSize(); i++) {
            float m = aiMOWMOTCURRENT.read();
            myMedian.add( m );
			delay(50);
        }
        myMedian.getAverage(8,offset); // Get Median of 8 values in the middle of the sorted array. Spikes are on the outside of the sorted array.

		errorHandler.setInfo(F("!03,Mow Motor Sensor Offset: %f\r\n"),offset);
     }
};

#endif
