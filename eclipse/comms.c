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
 * @brief   read from  USB_PORT or UART_PORT a string
 *
 * @param[in] fmt       formatting string
 * param[in] string		pointer to the beggining of where characters should be stored
 * param[in] numCharsToRead		max number of chars to read
 * param[in] endChar		optionnal input character whil will end reading
 *
 *@return              Number of chars read
 */
//void comms_readf(BaseSequentialStream *chp, char *string, uint16_t numCharsToRead, char endChar){
//
//}
