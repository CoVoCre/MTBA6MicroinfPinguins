/*
 * comms.c
 *
 *  Created on: May 7, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: Provides necessary interaction with computer for the project
 * 		this outputs via serial UART, which means either bluetooth or usb
 *
 * Functions prefix for public functions in this file: comms_
 */

#include <comms.h>
#include <usbcfg.h>
#include <chstreams.h>


/*===========================================================================*/
/* Constants definition for this file						               */
/*===========================================================================*/

//We redefine chibios elements for facilitated usage
#define UART_PORT SD3
#define UART_PORT_STREAM ((BaseSequentialStream *)&UART_PORT)

/*
 * From https://upload.wikimedia.org/wikipedia/commons/d/dd/ASCII-Table.svg
 * we define the different bounds and characters needed
 */
#define ASCII_NORMAL_TEXT_BEGIN 	33
#define ASCII_NORMAL_TEXT_END 	126
#define ASCII_DELETE_CHARACTER	127

//Number constants
#define TWO						2



/*===========================================================================*/
/* Static variables definitions 		 			                            */
/*===========================================================================*/

static bool comms_started = false;


/*===========================================================================*/
/* Public functions for setting/getting internal parameters           	  */
/*===========================================================================*/

void comms_start(void)
{
	if(comms_started == false){
		static SerialConfig ser_cfg = {
			    115200,
			    0,
			    0,
			    0,
			};

			//starts the serial communication on UART
			sdStart(&UART_PORT, &ser_cfg);
	}
	comms_started = true;
}

int comms_printf(const char *fmt, ...) {
	va_list ap;
	int formatted_bytes;

	va_start(ap, fmt);
	formatted_bytes = chvprintf(UART_PORT_STREAM, fmt, ap);
	va_end(ap);

	return formatted_bytes;
}

uint16_t comms_readf(char *readText, uint16_t arraySize){
	uint16_t numOfCharsRead = 0;
	char readChar;

	for(uint16_t i = 0; i<arraySize-1;i++){
		readChar = chSequentialStreamGet(UART_PORT_STREAM);
		switch(readChar){
		case '\n': 									//for either \n or \n end string and return
		case '\r':
			readText[i] = '\0';
			comms_printf(" \n\r");
			return numOfCharsRead = i-1;				// we do not count \0
		case ASCII_DELETE_CHARACTER:					//ASCII special delete character
				readText[i-1] = '\0';
				readText[i] = '\0';
				i-=TWO;								//-2 because we want to re read i-1 and for will increment i
				comms_printf("\r                                   \r%s",readText);	//erase line
			break;
		default:
													//We protect against special ASCII characters so we do nothing for them
			if( ASCII_NORMAL_TEXT_BEGIN <= readChar && readChar <= ASCII_NORMAL_TEXT_END){
				readText[i] = readChar;
				readText[i+1]= '\0';
				//we erase line before writing because comms_printf adds space between characters otherwise
				comms_printf("\r                                    \r%s",readText);
			}
		}
	}

	comms_printf(" \n\r");
	return numOfCharsRead;
}
