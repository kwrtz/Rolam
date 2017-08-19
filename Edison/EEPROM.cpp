// 
// 
// 

#include "EEPROM.h"

union uFloat {
	uint8_t uBytes[4];
	float sFloat;
};

union uInt32 {
	uint8_t uBytes[4];
	int32_t sIn32t;
};

union uInt16 {
	uint8_t uBytes[2];
	int16_t sIn16t;
};

union uInt8 {
	uint8_t uBytes[1];
	int8_t sIn8t;
};

TEEPROM::TEEPROM() {
	
}

void TEEPROM::setup() {
}

void TEEPROM::run() {
	runned();

}
void TEEPROM::write(uint16_t address, uint8_t data) {
	i2cEEPROM.write16(address, data);
	delay(20);
}

void TEEPROM::writeFloat(uint16_t address, float data) {
	uFloat f;
	f.sFloat = data;
	//errorHandler.setInfoNoLog(F("inputf: %f\r\n"), f.sFloat);
	write(address, f.uBytes[3]);
	//errorHandler.setInfoNoLog(F("w3: %d\r\n"), f.uBytes[3]);
	write(address+1, f.uBytes[2]);
	//errorHandler.setInfoNoLog(F("w2: %d\r\n"), f.uBytes[2]);
	write(address+2, f.uBytes[1]);
	//errorHandler.setInfoNoLog(F("w1: %d\r\n"), f.uBytes[1]);
	write(address+3, f.uBytes[0]);
	//errorHandler.setInfoNoLog(F("w0: %d\r\n"), f.uBytes[0]);
}

void TEEPROM::write32t(uint16_t address, int32_t data) {
	uInt32 i;
	i.sIn32t = data;
	//errorHandler.setInfoNoLog(F("input32t: %d\r\n"), i.sIn32t);
	write(address, i.uBytes[3]);
	write(address + 1, i.uBytes[2]);
	write(address + 2, i.uBytes[1]);
	write(address + 3, i.uBytes[0]);
}

void TEEPROM::write16t(uint16_t address, int16_t data) {
	uInt16 i;
	i.sIn16t = data;
	write(address, i.uBytes[1]);
	write(address + 1, i.uBytes[0]);
}

int TEEPROM::read(uint16_t address, uint8_t& data) {
	return i2cEEPROM.read16(address, data);
}

int TEEPROM::readFloat(uint16_t address, float& data) {
	uFloat f;
	int j = 0;
	j = read(address, f.uBytes[3]);
	//errorHandler.setInfoNoLog(F("r3: %d\r\n"), f.uBytes[3]);
	j += read(address + 1, f.uBytes[2]);
	//errorHandler.setInfoNoLog(F("r2: %d\r\n"), f.uBytes[2]);
	j += read(address + 2, f.uBytes[1]);
	//errorHandler.setInfoNoLog(F("r1: %d\r\n"), f.uBytes[1]);
	j += read(address + 3, f.uBytes[0]);
	//errorHandler.setInfoNoLog(F("r0: %d\r\n"), f.uBytes[0]);
	data = f.sFloat;
	if (j != 4) {
		return 0;
	}
	return j;
}

int TEEPROM::read32t(uint16_t address, int32_t& data) {
	uInt32 i;
	int j=0;
	j = read(address, i.uBytes[3]);
	j +=read(address + 1, i.uBytes[2]);
	j +=read(address + 2, i.uBytes[1]);
	j +=read(address + 3, i.uBytes[0]);
	data = i.sIn32t;
	if (j != 4) {
		return 0;
	}
	return j;
}



int TEEPROM::read16t(uint16_t address, int16_t& data) {
	uInt16 i;
	int j = 0;
	j = read(address, i.uBytes[1]);
	j += read(address + 1, i.uBytes[0]);
	data = i.sIn16t;
	if (j != 2) {
		return 0;
	}
	return j;
}