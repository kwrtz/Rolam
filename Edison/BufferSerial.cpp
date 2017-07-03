
#include "BufferSerial.h"



BufferSerial::BufferSerial(HardwareSerial& s, const int& bufferSize) : serial(s)
{
	_setup(bufferSize);
}


BufferSerial :: ~BufferSerial()
{
	delete[] _buf;
}

void BufferSerial::_setup(const int& bufferSize)
{
	if (bufferSize > 1) {
		_buf = new char[bufferSize];
		_size = bufferSize - 1;
	}
	else {
		_buf = new char[256];
		_size = 256 - 1;
	}
	_present = 0;
	_last = 0;

}

int BufferSerial::_getShift(volatile const int& value)
{
	return value ? (value - 1) : _size;
}

void BufferSerial::run(void)
{
	while (serial.available()) {
		int n = _getShift(_last);
		_buf[n] = (char)serial.read();
		_last = n;
	}
}

bool BufferSerial::unreadable(void)
{
	return (_present == _last);
}

bool BufferSerial::readable(void)
{
	return !unreadable();
}

char BufferSerial::getChar(void)
{
	if (unreadable()) {
		return -1;
	}
	else {
		_present = _getShift(_present);
		return (char)_buf[_present];
	}
}

/*
int BufferSerial::printf(char *str, ...)
{
	return 1;
	
}
*/

