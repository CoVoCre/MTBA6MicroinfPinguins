/*
 * comms.c
 *
 *  Created on: Apr 1, 2020
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

static bool comms_started = false;

/**
 * @brief   starts all communication things
 */
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

/**
 * @brief   use same function as chprintf() from chibios for sending information : System formatted output function.
 * @details This function implements a minimal @p printf() like functionality
 *          with output on a @p BaseSequentialStream.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *
 * @param[in] fmt       formatting string
 *
 *@return              The number of bytes that would have been
 *                      written to @p chp if no stream error occurs
 */
int comms_printf(const char *fmt, ...) {
	BaseSequentialStream *chp = UART_PORT_STREAM;
	va_list ap;
	int formatted_bytes;

	va_start(ap, fmt);
	formatted_bytes = chvprintf(chp, fmt, ap);
	va_end(ap);

	return formatted_bytes;
}

/**
 * @brief  	read from  USB_PORT or UART_PORT and store in char array
 * @note 	this function will print back what the user enters as he enters
 * 				so that he sees what he inputs
 * @warning	this function is blocking for the calling thread until either
 * 				the end char is met or the user presses enter
 *
 * @param[out] readText				pointer to char array where characters should be stored
 * 										\0 will be put at the end of table after reading
 * @param[in] arraySize				dictates max number of chars to be read including last \0 end character
 *
 *@return	Number of chars read and stored in array (not counting \0)
 */
uint16_t comms_readf(char *readText, uint16_t arraySize){
	BaseSequentialStream *in = UART_PORT_STREAM;
	uint16_t numOfCharsRead = 0;
	char readChar;

	for(uint16_t i = 0; i<arraySize-1;i++){
		readChar = chSequentialStreamGet(in);
		switch(readChar){
		case '\n': //for either \n or \n end string and return
		case '\r':
			readText[i] = '\0';
			comms_printf(in," \n\r");
			return numOfCharsRead = i-1;	// we do not count \0
		case 127:	//ASCII special delete character
				readText[i-1] = '\0';
				readText[i] = '\0';
				i-=1;
				comms_printf(in,"\r                                   \r%s",readText);	//erase line
			break;
		default:
			//We protect against special ASCII characters so we do nothing for them
			if( ASCII_NORMAL_TEXT_BEGIN <= readChar && readChar <= ASCII_NORMAL_TEXT_BEGIN){
				readText[i] = readChar;
				readText[i+1]= '\0';
				//we erase line before writing because comms_printf adds space between characters otherwise
				comms_printf(in,"\r                                    \r%s",readText);
			}
		}
	}

	comms_printf(in," \n\r");
	return numOfCharsRead;
}
