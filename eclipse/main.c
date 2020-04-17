#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

#include <main.h>

#include <travelController.h>
#include <comms.h>

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

#define NB_BYTE_PER_CMPX_VAL			2


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

	comms_start();

	//chThdSleepMilliseconds(100);

	comms_printf(UART_PORT_STREAM, "In MAIN \n\r");
	comms_printf(UART_PORT_STREAM, "We will now ask for text. Enter anything then press enter\n\r");

	uint8_t textByUser[100] = "Default string";

	comms_printf(UART_PORT_STREAM, "  the default string is : %s \n\r", textByUser);

    /* Infinite loop. */
    while (1) {
    	comms_printf(UART_PORT_STREAM, "Now please enter anything\n\r");

    	comms_readf(UART_PORT_STREAM, textByUser, 100 );

    	comms_printf(UART_PORT_STREAM, "The text you just entered : %s\n\r", textByUser);
    	comms_printf(UART_PORT_STREAM, " \n\r");
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
