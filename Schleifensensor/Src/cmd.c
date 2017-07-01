/*******************************************************************
 Copyright (C) 2009 FreakLabs
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 3. Neither the name of the the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 SUCH DAMAGE.

 Originally written by Christopher Wang aka Akiba.
 Please post support questions to the FreakLabs forum.

 *******************************************************************/
/*!
 \file Cmd.c

 This implements a simple command line interface for the Arduino so that
 its possible to execute individual functions within the sketch.

Adapted to use Cube Hal by Kai Würtz 2016
Thanks to HammerFet
http://stackoverflow.com/questions/24875873/stm32f4-uart-hal-driver
 */
/**************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Cmd.h"

extern UART_HandleTypeDef huart2;

void cmd_parse(char *cmd);

// command line message buffer
static uint8_t rxBuffer = '\000'; // where we store that one character that just came in
static uint8_t rxString[MAX_MSG_SIZE]; // where we build our string from characters coming in
static int rxindex = 0; // index for going though rxString

// linked list for command table
static cmd_t *cmd_tbl_list, *cmd_tbl;

// text strings for command prompt (stored in flash)
//const char cmd_banner[] PROGMEM = "*************** CMD *******************";
//const char cmd_prompt[] PROGMEM = "CMD >> ";
const char cmd_unrecog[] = "CMD: Command not recognized.\r\n";

/**************************************************************************/
/*!
 Print one char to serial line
 */
/**************************************************************************/
void print2(char c) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &c, 1, HAL_MAX_DELAY);
}

/**************************************************************************/
/*!
 Interrupt callback routine.
 Will be called every time a char arises
 */
/**************************************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART2)	//current UART
	{

  		   __HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun

		    int i = 0;

		    if (rxBuffer == 8 || rxBuffer == 127) // If Backspace or del
		    {

		    	//print(" \b"); // "\b space \b" clears the terminal character. Remember we just echoced a \b so don't need another one here, just space and \b
		        //rxindex--;
		        //if (rxindex < 0) rxindex = 0;
		    }

		    else if (rxBuffer == '\r') // If Enter
		    {
		        rxString[rxindex] = '\0';
		    	cmd_parse((char *) rxString);
		        rxindex = 0;
		        for (i = 0; i < MAX_MSG_SIZE; i++) rxString[i] = 0; // Clear the string buffer
		    }
		    else if (rxBuffer == '\n') // If Enter
		    {
		    }
		    else if(  rxBuffer == 0x3){
		    	// End of text überlesen
		    }
		    else
		    {
		        rxString[rxindex] = rxBuffer; // Add that character to the string
		        rxindex++;
			    print2(rxBuffer); // Echo the character that caused this callback so the user can see what they are typing
		        if (rxindex > MAX_MSG_SIZE) // User typing too much, we can't have commands that big
		        {
		            rxindex = 0;
		            for (i = 0; i < MAX_MSG_SIZE; i++) rxString[i] = 0; // Clear the string buffer
				    print2('L');print2('O');print2('N');print2('G');
		            print2('\r');
		            print2('\n');
		        }
		    }
	}
}


/**************************************************************************/
/*!
 Parse the command line. This function tokenizes the command input, then
 searches for the command table entry associated with the commmand. Once found,
 it will jump to the corresponding function.
 */
/**************************************************************************/
void cmd_parse(char *cmd) {
	uint8_t argc, i = 0;
	char *argv[30];
	cmd_t *cmd_entry;

	//debug.flush();
	//fflush(stdout);

	// parse the command line statement and break it up into space-delimited
	// strings. the array of strings will be saved in the argv array.
	argv[i] = strtok(cmd, " ");
	do {
		argv[++i] = strtok(NULL, " ");
	} while ((i < 30) && (argv[i] != NULL));

	// save off the number of arguments for the particular command.
	argc = i;

	// parse the command table for valid command. used argv[0] which is the
	// actual command name typed in at the prompt
	for (cmd_entry = cmd_tbl; cmd_entry != NULL; cmd_entry = cmd_entry->next) {
		//println(argv[0]);
		if (!strcmp(argv[0], cmd_entry->cmd)) {
			cmd_entry->func(argc, argv);
			return;
		}
	}

	// command not recognized. print message and re-generate prompt.
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd_unrecog, strlen(cmd_unrecog),
			HAL_MAX_DELAY);
}


/**************************************************************************/
/*!
 Initialize the command line interface. This sets the terminal speed and
 and initializes things.
 */
/**************************************************************************/
void cmdInit() {
	rxindex = 0;

	// init the command table
	cmd_tbl_list = NULL;

	//HAL_UART_Receive_IT(&huart2, Rx_data, 1);
	__HAL_UART_FLUSH_DRREGISTER(&huart2);
	HAL_UART_Receive_DMA(&huart2, &rxBuffer, 1);

}

/**************************************************************************/
/*!
 Add a command to the command table. The commands should be added in
 at the setup() portion of the sketch.
 */
/**************************************************************************/
void cmdAdd(char *name, void (*func)(int argc, char **argv)) {
	// alloc memory for command struct
	cmd_tbl = (cmd_t *) malloc(sizeof(cmd_t));

	// alloc memory for command name
	char *cmd_name = (char *) malloc(strlen(name) + 1);

	// copy command name
	strcpy(cmd_name, name);

	// terminate the command name
	cmd_name[strlen(name)] = '\0';

	// fill out structure
	cmd_tbl->cmd = cmd_name;
	cmd_tbl->func = func;
	cmd_tbl->next = cmd_tbl_list;
	cmd_tbl_list = cmd_tbl;
}

/**************************************************************************/
/*!
 Convert a string to a number. The base must be specified, ie: "32" is a
 different value in base 10 (decimal) and base 16 (hexadecimal).
 */
/**************************************************************************/
uint32_t cmdStr2Num(char *str, uint8_t base) {
	return strtol(str, NULL, base);
}

float cmdStr2Float(char *str) {
	return atof(str);
}

