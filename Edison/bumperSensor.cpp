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

#include "bumperSensor.h"
#include "motor.h"

// Motorensteuerung Interface. KEIN THREAD. Wird verwendet um clcX und pcX zu steuern.
extern TMotorInterface motor;

void   TbumperSensor::setup()
{
    int i;

    _bumperActivated = false;
    flagShowBumper = false;

    for(i=0; i < MPX5010DP_BUF_SIZE; i++) {
        _buf[i] =  aiBumper.read();
    }
};

bool TbumperSensor::isBumperActivated()
{
    return _bumperActivated;
}


void TbumperSensor::run()
{
    int i;
    float sum1=0.0f;
    float sum2=0.0f;
    float sum3=0.0f;
    float sum4=0.0f;
    float sum5=0.0f;
    float sumReading=0.0f;

	runned();

    memcpy(&_buf[0],&_buf[1],sizeof(float) * (MPX5010DP_BUF_SIZE-1));
	//memmove(_buf, _buf+1, 196);

	

    //_buf[ MPX5010DP_BUF_SIZE-1 ] = aiBumper.read();
    //y[i] := y[i-1] + a * (x[i] - y[i-1])
    _buf[ MPX5010DP_BUF_SIZE-1 ] = _buf[ MPX5010DP_BUF_SIZE-2 ]  + 0.25f *( aiBumper.read() - _buf[ MPX5010DP_BUF_SIZE-2 ]  );

    //debug->printf("%f ",   _buf[ MPX5010DP_BUF_SIZE-1 ] );


    
    //    pc.printf("\r\n#######\r\n");
    //    for(i=0; i < MPX5010DP_BUF_SIZE; i++) {
    //        pc.printf("%f ", _buf[i]);
    //    }
    //    pc.printf("\r\n#######\r\n");
    


    for(i=0; i < 4; i++) {
        sum1 += _buf[i];
        //pc.printf("sum1 %f ", _buf[i]);
    }
    sum1 /=4;


    for(i=MPX5010DP_BUF_SIZE-32; i < MPX5010DP_BUF_SIZE-28; i++) {
        sum5 += _buf[i];
        //pc.printf("sum4 %f ", _buf[i]);
    }
    sum5 /=4;


    for(i=MPX5010DP_BUF_SIZE-24; i < MPX5010DP_BUF_SIZE-20; i++) {
        sum4 += _buf[i];
        //pc.printf("sum4 %f ", _buf[i]);
    }
    sum4 /=4;

    for(i=MPX5010DP_BUF_SIZE-16; i < MPX5010DP_BUF_SIZE-12; i++) {
        sum3 += _buf[i];
        //pc.printf("sum3 %f ", _buf[i]);
    }
    sum3 /=4;

    for(i=MPX5010DP_BUF_SIZE-8; i < MPX5010DP_BUF_SIZE-4; i++) {
        sum2 += _buf[i];
        //pc.printf("sum2 %f ", _buf[i]);
    }
    sum2 /=4;

    for(i=MPX5010DP_BUF_SIZE-2; i < MPX5010DP_BUF_SIZE; i++) {
        sumReading += _buf[i];
        //pc.printf("sumReading %f ", _buf[i]);
    }
    sumReading /=2;



    if(!_bumperActivated) {
        if(sumReading-sum1 > 0.04f) {
            _bumperActivated = true;
            if(flagShowBumper) {
               sprintf(errorHandler.msg,"!03,reading-1: %f-%f\r\n", sumReading,sum1);
               errorHandler.setInfo(); 
            }
        }
        if(sumReading-sum2 > 0.04f) {
            _bumperActivated = true;
            if(flagShowBumper) {
                sprintf(errorHandler.msg,"!03,reading-2: %f-%f\r\n", sumReading,sum2);
                     errorHandler.setInfo(); 
            }
        }
        if(sumReading-sum3 > 0.04f) {
            _bumperActivated = true;
            if(flagShowBumper) {
                sprintf(errorHandler.msg,"!03,reading-3: %f-%f\r\n", sumReading,sum3);
                     errorHandler.setInfo(); 
            }
        }
        if(sumReading-sum4 > 0.04f) {
            _bumperActivated = true;
            if(flagShowBumper) {
                sprintf(errorHandler.msg,"!03,reading-4: %f-%f\r\n", sumReading,sum4);
                     errorHandler.setInfo(); 
            }
        }
        if(sumReading-sum5 > 0.04f) {
            _bumperActivated = true;
            if(flagShowBumper) {
                sprintf(errorHandler.msg,"!03,reading-5: %f-%f\r\n", sumReading,sum5);
                     errorHandler.setInfo(); 
            }
        }


        // calculate deactivate threshold out of least sum
        if(_bumperActivated) {

            motor.hardStop();

            _deactivateThreshold = sum1;

            if(sum2 <_deactivateThreshold)
                _deactivateThreshold = sum2;

            if(sum3 <_deactivateThreshold)
                _deactivateThreshold = sum3;

            if(sum4 <_deactivateThreshold)
                _deactivateThreshold = sum4;

            if(sum5 <_deactivateThreshold)
                _deactivateThreshold = sum5;

            _deactivateThreshold += 0.02f;

            if(flagShowBumper) {
                sprintf(errorHandler.msg,"!03,_deactivateThreshold: %f\r\n", _deactivateThreshold);
                     errorHandler.setInfo();


                errorHandler.setInfo("!03,\r\n#######\r\n");
                for(i=0; i < MPX5010DP_BUF_SIZE; i++) {
                    sprintf(errorHandler.msg,"!03,%f ", _buf[i]);
                     errorHandler.setInfo(); 
                }
                errorHandler.setInfo("!03,\r\n#######\r\n");


            }
        }


    } 
	
	else {
        if(sumReading < _deactivateThreshold) {
            _bumperActivated = false;
            if(flagShowBumper) {
                sprintf(errorHandler.msg,"!03,reading-deacThreshold: %f-%f\r\n",sumReading, _deactivateThreshold);
                errorHandler.setInfo(); 
            }
            // Reset array to new measured value because current values are bumper pressed values
            for(i=0; i < MPX5010DP_BUF_SIZE-2; i++) {
                _buf[i] = sumReading;
            }
        }

    }


    //pc.printf("#######\r\n");
};



