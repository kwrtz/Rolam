// CRotaryEncoder.h
// https://developer.mbed.org/users/Raabinator/code/CRotaryEncoder/
// Changed for own needs 2017 Kai Würtz
// 

#ifndef _CROTARYENCODER_h
#define _CROTARYENCODER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "DigitalInOut.h"

/* This Class handles a rotary encoder like the one from Pollin electronic (Panasonic EVEP...).
* It uses two pins, one creating an interrupt on change.
* Rotation direction is determined by checking the state of the other pin.
*
* Operating the encoder changes an internal integer value that can be read
* by Get() or the operator int() functions.
* A new value can be set by Set(value) or opperator=.
*
* Autor: Thomas Raab (Raabinator)
*
* Dent steady point     !     !     !
*                    +-----+     +-----+
* pinA (interrupt)   |     |     |     |
*                  --+     +-----+     +---
*                      +-----+     +-----+
* pinB                 |     |     |     |
*                  ----+     +-----+     +-
*                           --> C.W
* CW:  increases position value
* CCW: decreases position value
*
* changelog:
*
* 09. Nov. 2010
*     First version published.
*
*/



class CRotaryEncoder
{
public:
	CRotaryEncoder(DigitalIn &_pinA, DigitalIn &_pinB);
	~CRotaryEncoder();

	void isReversed();

	long getTickCounter();
	void resetTickCounter();
	unsigned long getAbsTicksCounter();
	void resetAbsTicksCounter();

	void rise(void);
	void fall(void);

private:
	DigitalIn &m_pinA;
	DigitalIn &m_pinB;
	volatile long    m_ticks;     // Counts +/-
	volatile unsigned long  m_abs_ticks;     //  // Counts + only
	bool _isReversed;




};


#endif

