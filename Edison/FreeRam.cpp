// 
// https://forum.arduino.cc/index.php?topic=381203.0
// https://forum.arduino.cc/index.php?topic=404908.0
// https://forum.pjrc.com/threads/33443-How-to-display-free-ram
// 

#include "FreeRam.h"
#include "hardware.h"

#include <malloc.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  

extern char _end;

void FreeMem(void) {
	char *ramstart = (char *)0x20070000;
	char *ramend = (char *)0x20088000;

	char Txt[256];


	char *heapend = sbrk(0);
	register char * stack_ptr asm("sp");
	struct mallinfo mi = mallinfo();

	unsigned int* ptr_i;
	unsigned int i = 0xaa;
	ptr_i = &i;

	sprintf(Txt, "Size of int: %d, Size of uint: %d\n", sizeof(int), sizeof(unsigned int)); debug.serial.print(Txt);
	sprintf(Txt, "Size of char: %d, Size of uchar: %d\n", sizeof(char), sizeof(unsigned char)); debug.serial.print(Txt);
	sprintf(Txt, "Size of short: %d, Size of ushort: %d\n", sizeof(short), sizeof(unsigned short)); debug.serial.print(Txt);
	sprintf(Txt, "Size of float: %d, Size of double: %d\n", sizeof(float), sizeof(double)); debug.serial.print(Txt);
	sprintf(Txt, "Size of long: %d, Size of longlong: %d\n", sizeof(long), sizeof(long long)); debug.serial.print(Txt);
	sprintf(Txt, "Size of pointer (ptr_i): %d\n", sizeof(ptr_i)); debug.serial.print(Txt);
	sprintf(Txt, "Addr of pointer (ptr_i): 0x%x\n", (unsigned int)ptr_i); debug.serial.print(Txt);
	sprintf(Txt, "Value of i: 0x%x\n", *ptr_i); debug.serial.print(Txt);


	sprintf(Txt, "    arena = %d\n", mi.arena);     debug.serial.print(Txt);
	sprintf(Txt, "  ordblks = %d\n", mi.ordblks);   debug.serial.print(Txt);
	sprintf(Txt, " uordblks = %d\n", mi.uordblks);  debug.serial.print(Txt);
	sprintf(Txt, " fordblks = %d\n", mi.fordblks);  debug.serial.print(Txt);
	sprintf(Txt, " keepcost = %d\n", mi.keepcost);  debug.serial.println(Txt);

	sprintf(Txt, "RAM Start:    %lx\n", (unsigned long)ramstart);  Serial.print(Txt);
	sprintf(Txt, "Data/Bss end: %lx\n", (unsigned long)&_end);     Serial.print(Txt);
	sprintf(Txt, "Heap End:     %lx\n", (unsigned long)heapend);   Serial.print(Txt);
	sprintf(Txt, "Stack Ptr:    %lx\n", (unsigned long)stack_ptr); Serial.print(Txt);
	sprintf(Txt, "RAM End:      %lx\n", (unsigned long)ramend);    Serial.println(Txt);

	sprintf(Txt, "Heap RAM Used:      %d\n", mi.uordblks);                       Serial.print(Txt);
	sprintf(Txt, "Program RAM Used:   %d\n", &_end - ramstart);                  Serial.print(Txt);
	sprintf(Txt, "Stack RAM Used:     %d\n", ramend - stack_ptr);                Serial.print(Txt);
	sprintf(Txt, "Estimated Free RAM: %d\n", stack_ptr - heapend + mi.fordblks); Serial.print(Txt);

	/* add main program code here */
	uint32_t stackTop;
	uint32_t heapTop;

	// current position of the stack.
	stackTop = (uint32_t)&stackTop;

	// current position of heap.
	void* hTop = malloc(1);
	heapTop = (uint32_t)hTop;
	free(hTop);
	// The difference is the free, available ram.
	sprintf(Txt, "Estimated Free RAM2:%lu\n", stackTop - heapTop); Serial.print(Txt);
	char top;
	sprintf(Txt, "Estimated Free RAM3:%d\n", &top - reinterpret_cast<char*>(sbrk(0))); Serial.print(Txt);

}