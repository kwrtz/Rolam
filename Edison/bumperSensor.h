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

#ifndef MPX5010DP_H_INCLUDED
#define MPX5010DP_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Thread.h"
#include "helpers.h"
#include "hardware.h"

#define MPX5010DP_BUF_SIZE 50  // Bei 15ms Intervall sind dies 750ms die zurÃ¼ckgeguckt werden

class TbumperSensor : public Thread
{
public:
    bool flagShowBumper;
    void setup();
    virtual void run();
    bool isBumperActivated();
private:
    bool _bumperActivated;
	float _deactivateThreshold;
	float _buf[MPX5010DP_BUF_SIZE];

};


#endif

