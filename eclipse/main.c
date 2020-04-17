#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <main.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>


#include <travelController.h>
#include <comms.h>

static void timer12_start(void){
    //General Purpose Timer configuration
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void)
{
	halInit(); //
	chSysInit();
	mpu_init();
	timer12_start();
	comms_start();
	chprintf(UART_PORT_STREAM,"Starting main !\n\r");

//	motors_init();
//	chThdSleepMilliseconds(100);
//	right_motor_set_speed(200);
//	left_motor_set_speed(200);

	travCtrl_testAll();
	while(1){
		//chprintf(UART_PORT_STREAM, "In MAIN while loop\n\r");
		chThdSleepMilliseconds(100);
	}
}

/*===========================================================================*/
/* Something to protect agains something...                                  */
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
