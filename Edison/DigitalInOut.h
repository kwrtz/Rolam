/* mbed Microcontroller Library
* Copyright (c) 2006-2013 ARM Limited
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* Adapted to Arduion 2017 Kai Würtz
*
*/


#ifndef _DIGITALINOUT_h
#define _DIGITALINOUT_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

inline void digitalWriteDirect(int pin, int val) {
	if (val) g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;
	else    g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin;
}


inline int digitalReadDirect(int pin) {
	return !!(g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin);
}



/** \addtogroup drivers */

/** A digital input, used for reading the state of a pin
*
* @note Synchronization level: Interrupt safe
*
* Example:
* @code
* // Flash an LED while a DigitalIn is true
*
* #include "mbed.h"
*
* DigitalIn enable(p5);
* DigitalOut led(LED1);
*
* int main() {
*     while(1) {
*         if(enable) {
*             led = !led;
*         }
*         wait(0.25);
*     }
* }
* @endcode
* @ingroup drivers
*/

class DigitalIn {

public:
	/** Create a DigitalIn connected to the specified pin
	*
	*  @param pin DigitalIn pin to connect to
	*/
	DigitalIn(const uint8_t pin, boolean pullup) : myPin(pin) {
		// No lock needed in the constructor

		if (pullup) {
			pinMode(pin, INPUT_PULLUP);
		}
		else {
			pinMode(pin, INPUT);
		}

	}


	/** Read the input, represented as 0 or 1 (int)
	*
	*  @returns
	*    An integer representing the state of the input pin,
	*    0 for logical 0, 1 for logical 1
	*/
	int read() {
		return digitalRead(myPin);
	}
	int readDirect() {
		return digitalReadDirect(myPin);
	}

	/** An operator shorthand for read()
	* \sa DigitalIn::read()
	*/
	operator int() {
		return read();
	}

	uint8_t pin() {
		return myPin;
	}

protected:
	uint8_t myPin;
};


/** A digital output, used for setting the state of a pin
*
* @note Synchronization level: Interrupt safe
*
* Example:
* @code
* // Toggle a LED
* #include "mbed.h"
*
* DigitalOut led(LED1);
*
* int main() {
*     while(1) {
*         led = !led;
*         wait(0.2);
*     }
* }
* @endcode
* @ingroup drivers
*/
class DigitalOut {

public:
	/** Create a DigitalOut connected to the specified pin
	*
	*  @param pin DigitalOut pin to connect to
	*/
	DigitalOut(const uint8_t pin) : myPin(pin) {
		// No lock needed in the constructor
		pinMode(pin, OUTPUT);
		write(LOW);
	}
	/** Create a DigitalOut connected to the specified pin
	*
	*  @param pin DigitalOut pin to connect to
	*  @param value the initial pin value
	*/
	DigitalOut(const uint8_t pin, int value) : myPin(pin) {
		pinMode(pin, OUTPUT);
		write(value);

	}

	/** Set the output, specified as 0 or 1 (int)
	*
	*  @param value An integer specifying the pin output value,
	*      0 for logical 0, 1 (or any other non-zero value) for logical 1
	*/
	void write(int value) {
		digitalWrite(myPin, value);
		myValue = value;
	}
	void writeDirect(int value) {
		digitalWriteDirect(myPin, value);
		myValue = value;
	}

	/** Return the output setting, represented as HIGH | LOW (int)
	*
	*  @returns
	*    an integer representing the output setting of the pin,
	*    0 for logical 0, 1 for logical 1
	*/
	int read() {
		return myValue;
	}


	/** A shorthand for write()
	* \sa DigitalOut::write()
	*/
	DigitalOut& operator= (int value) {
		write(value);
		return *this;
	}


	/** A shorthand for write()
	* \sa DigitalOut::write()
	*/
	DigitalOut& operator= (DigitalOut& rhs) {
		write(rhs.read());
		return *this;
	}

	/** A shorthand for read()
	* \sa DigitalOut::read()
	*/
	operator int() {
		// Underlying call is thread safe
		return read();
	}

	uint8_t pin() {
		return myPin;
	}
protected:
	uint8_t myPin;
	int myValue;
};

/** \addtogroup drivers */

/** An analog input, used for reading the voltage on a pin
*
* @note Synchronization level: Thread safe
*
* Example:
* @code
* // Print messages when the AnalogIn is greater than 50%
*
* #include "mbed.h"
*
* AnalogIn temperature(p20);
*
* int main() {
*     while(1) {
*         if(temperature > 0.5) {
*             printf("Too hot! (%f)", temperature.read());
*         }
*     }
* }
* @endcode
* @ingroup drivers
*/
class AnalogIn {

public:

	/** Create an AnalogIn, connected to the specified pin
	*
	* @param pin AnalogIn pin to connect to
	*/
	AnalogIn(const uint8_t pin) : myPin(pin) {
		pinMode(pin, INPUT);
	}

	/** Read the input voltage, represented as a float in the range [0.0, 1.0]
	*
	* @returns A floating-point value representing the current input voltage, measured as a percentage
	*/
	float read() {
		float ret = (float)read_u16() / 1023.0f;
		return ret;
	}

	/** Read the input voltage, represented as an unsigned short in the range [0x0, 0xFFFF]
	*
	* @returns
	*   16-bit unsigned short representing the current input voltage, normalised to a 16-bit value
	*/
	unsigned short read_u16() {
		unsigned short ret = analogRead(myPin);
		return ret;
	}

	/** An operator shorthand for read()
	*
	* The float() operator can be used as a shorthand for read() to simplify common code sequences
	*
	* Example:
	* @code
	* float x = volume.read();
	* float x = volume;
	*
	* if(volume.read() > 0.25) { ... }
	* if(volume > 0.25) { ... }
	* @endcode
	*/
	operator float() {
		return read();
	}

	virtual ~AnalogIn() {
		// Do nothing
	}
	
	uint8_t pin() {
		return myPin;
	}


protected:
	uint8_t myPin;
};


#endif

