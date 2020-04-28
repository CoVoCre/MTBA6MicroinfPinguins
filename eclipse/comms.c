/*
 * comms.c
 *
 *  Created on: Apr 5, 2020
 *      Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: Provides necessary interaction with computer for the project
 * Functions prefix for this file: comms_
 */

#include <comms.h>
#include <usbcfg.h>
#include <chstreams.h>

static bool comms_started = false;

//chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
//                imu_values.acc_raw[X_AXIS], imu_values.acc_raw[Y_AXIS], imu_values.acc_raw[Z_AXIS],
//                imu_values.gyro_raw[X_AXIS], imu_values.gyro_raw[Y_AXIS], imu_values.gyro_raw[Z_AXIS]);

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
			//start the USB communication
			usb_start();
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
 * @param[in] chp       pointer to a @p BaseSequentialStream implementing object
 * @param[in] fmt       formatting string
 *
 *@return              The number of bytes that would have been
 *                      written to @p chp if no stream error occurs
 */
int comms_printf(BaseSequentialStream *chp, const char *fmt, ...) {
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
 * @param[in] in       				stream to read from
 * @param[out] readText				pointer to char array where characters should be stored
 * 										\0 will be put at the end of table after reading
 * @param[in] arraySize				dictates max number of chars to be read including last \0 end character
 *
 *@return	Number of chars read and stored in array (not counting \0)
 */
uint16_t comms_readf(BaseSequentialStream *in, char *readText, uint16_t arraySize){
	uint16_t numOfCharsRead = 0;
	char readChar;

	comms_printf(UART_PORT_STREAM, "In comms_readf\n\r");


	for(uint16_t i = 0; i<arraySize-1;i++){
		readChar = chSequentialStreamGet(in);
		switch(readChar){
		case '\n': //for either \n or \n end and return
		case '\r':
			readText[i] = '\0';
			comms_printf(in," \n\r");
			return numOfCharsRead = i-1;	// we do not count \0
		case 127:
				readText[i-1] = '\0';
				readText[i] = '\0';
				i-=1;
				comms_printf(in,"\r                                                                          \r%s",readText);
			break;
		default:
			readText[i] = readChar;
			readText[i+1]= '\0';
			comms_printf(in,"\r                                                                          \r%s",readText);
		}
	}

	comms_printf(in," \n\r");
	return numOfCharsRead;
}

//TESTPING
void comms_test_test(void){
	comms_printf(UART_PORT_STREAM, "In comms_test_test\n\r");
	uint8_t testChar = 'T';
	comms_printf(UART_PORT_STREAM,"First %c\n\r",testChar);
	testChar = 'E';
	comms_printf(UART_PORT_STREAM,"Second %c\n\r",testChar);
	testChar = 'S';
	chprintf(UART_PORT_STREAM,"%c ",testChar);
	testChar = 'T';
	chprintf(UART_PORT_STREAM,"Next %c ",testChar);
	testChar = '\r';
	chprintf(UART_PORT_STREAM,"%c ",testChar);
}
