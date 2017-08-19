// EEPROM.h

#ifndef _EEPROM_h
#define _EEPROM_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Thread.h"
#include "hardware.h"
#include "errorhandler.h"

class TEEPROM : public Thread {
public:
	TEEPROM();
	void setup();
	virtual void run();
	void write(uint16_t address, uint8_t data);
	void writeFloat(uint16_t address, float data);
	void write32t(uint16_t address, int32_t data);
	void write16t(uint16_t address, int16_t data);

	int read(uint16_t address, uint8_t& data);
	int readFloat(uint16_t address, float& data);
	int  read32t(uint16_t address, int32_t& data);
	int read16t(uint16_t address, int16_t& data);

protected:
	
private:
	
};

#endif

