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
#include <chprintf.h>

#define USB_PORT SDU1
#define UART_PORT SD3

//chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
//                imu_values.acc_raw[X_AXIS], imu_values.acc_raw[Y_AXIS], imu_values.acc_raw[Z_AXIS],
//                imu_values.gyro_raw[X_AXIS], imu_values.gyro_raw[Y_AXIS], imu_values.gyro_raw[Z_AXIS]);


static void comms_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};


	//starts the serial communication
	serial_start();
	//start the USB communication
	usb_start();

	sdStart(&UART_PORT, &ser_cfg);
	sdStart(&USB_PORT, &ser_cf);
}
