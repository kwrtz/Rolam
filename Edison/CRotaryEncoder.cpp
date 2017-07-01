// https://developer.mbed.org/users/Raabinator/code/CRotaryEncoder/
// Changed for own needs 2017 Kai Würtz
// 

#include "CRotaryEncoder.h"



CRotaryEncoder::CRotaryEncoder(DigitalIn &_pinA, DigitalIn &_pinB): m_pinA(_pinA), m_pinB(_pinB)
{

	m_ticks = 0;
	m_abs_ticks = 0;
	_isReversed = false;
}

CRotaryEncoder::~CRotaryEncoder()
{

}


long   CRotaryEncoder::getTickCounter()
{
	return m_ticks;
};

void CRotaryEncoder::resetTickCounter()
{
	m_ticks = 0;
};


unsigned long CRotaryEncoder::getAbsTicksCounter()
{
	return m_abs_ticks;
};


void CRotaryEncoder::resetAbsTicksCounter()
{
	m_abs_ticks = 0;
};



void CRotaryEncoder::isReversed()
{
	_isReversed = true;
}

/*
#ifdef LeftEncoderIsReversed
_LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
#else
_LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
#endif
*/


void CRotaryEncoder::fall(void)
{
	int b;

	if (_isReversed) {
		b = m_pinB.readDirect() ? -1 : +1;
	}
	else {
		b = m_pinB.readDirect() ? +1 : -1;
	}

	m_ticks += b;
	m_abs_ticks++;

}


void CRotaryEncoder::rise(void)
{
	int b;

	if (_isReversed) {
		b = m_pinB.readDirect() ? +1 : -1;
	}
	else {
		b = m_pinB.readDirect() ? -1 : +1;
	}

	m_ticks += b;
	m_abs_ticks++;

}
