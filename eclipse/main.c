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


/*Enable for Debugging main*/
#define DEBUG_MAIN

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

#define NB_BYTE_PER_CMPX_VAL			2

travCtrl_dirAngleCb_t updateAngle;

//static void timer12_start(void){
//    //General Purpose Timer configuration
//
//    //timer 12 is a 16 bit timer so we can measure time
//    //to about 65ms with a 1Mhz counter
//    static const GPTConfig gpt12cfg = {
//        1000000,        /* 1MHz timer clock in order to measure uS.*/
//        NULL,           /* Timer callback.*/
//        0,
//        0
//    };
//
//    gptStart(&GPTD12, &gpt12cfg);
//    //let the timer count to max value
//    gptStartContinuous(&GPTD12, 0xFFFF);
//}

void destReachedCB(void){
	//destReached = true;
	chprintf(UART_PORT_STREAM,"---------------------------------------------------------------\n\r");
	chprintf(UART_PORT_STREAM,"-                                                             -\n\r");
	chprintf(UART_PORT_STREAM,"-                                                             -\n\r");
	chprintf(UART_PORT_STREAM,"WARNING test_destReachedCB was called and waiting 1second\n\r");
	chprintf(UART_PORT_STREAM,"-                                                             -\n\r");
	chprintf(UART_PORT_STREAM,"-                                                             -\n\r");
	chprintf(UART_PORT_STREAM,"---------------------------------------------------------------\n\r");

	chThdSleepMilliseconds(1000);
}

int main(void)
{
	halInit(); //
	chSysInit();
	mpu_init();
	//timer12_start();
	comms_start();
	chprintf(UART_PORT_STREAM,"Starting main !\n\r");

	updateAngle = travCtrl_init(destReachedCB);


	//TESTPING test travelController functions!
	//travCtrl_testAll();

    mpu_init();

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it

    static float mic_data_right[NB_BYTE_PER_CMPX_VAL*FFT_SIZE];
    static float mic_data_left[NB_BYTE_PER_CMPX_VAL*FFT_SIZE];
    static float mic_data_front[NB_BYTE_PER_CMPX_VAL*FFT_SIZE];
    static float mic_data_back[NB_BYTE_PER_CMPX_VAL*FFT_SIZE];
    static float mic_ampli_right[FFT_SIZE];
    static float mic_ampli_left[FFT_SIZE];
    static float mic_ampli_front[FFT_SIZE];
    static float mic_ampli_back[FFT_SIZE];

  //  uint16_t source[NB_SOURCES]				= {0};
    int16_t arg								= 0;
    uint8_t	source_index						= 0;

    /* SEND_FROM_MIC */
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);


    /* Infinite loop. */
    while (1) {

        /*Waits until enough samples are collected*/
    		wait_send_to_computer();

        /*Copy buffer to avoid conflicts*/
        arm_copy_f32(get_audio_buffer_ptr(LEFT_CMPLX_INPUT), mic_data_left, NB_BYTE_PER_CMPX_VAL*FFT_SIZE);
        arm_copy_f32(get_audio_buffer_ptr(RIGHT_CMPLX_INPUT), mic_data_right, NB_BYTE_PER_CMPX_VAL*FFT_SIZE);
        arm_copy_f32(get_audio_buffer_ptr(FRONT_CMPLX_INPUT), mic_data_front, NB_BYTE_PER_CMPX_VAL*FFT_SIZE);
        arm_copy_f32(get_audio_buffer_ptr(BACK_CMPLX_INPUT), mic_data_back, NB_BYTE_PER_CMPX_VAL*FFT_SIZE);

        /*Determines peak freq of all sources*/
        //audioDetermineSource(mic_data_left, mic_ampli_left, source);
#ifdef DEBUG_MAIN
        /*for(uint8_t source_index=ZERO; source_index<NB_SOURCES; source_index++){
			if(source[source_index]==ERROR_AUDIO){
					chprintf((BaseSequentialStream *)&SD3, "Error in audioDetermineSource\n\r\n\r");
			}
			else{
					chprintf((BaseSequentialStream *)&SD3, "Source %d : 			%d\n\r", arg);
			}
        }*/
#endif

        /*Calculates angle of sound direction*/
        arg = audioAnalyseDirection(mic_data_left, mic_data_right, mic_data_back, mic_data_front,
    		   	   	   	   	   	   	   mic_ampli_left, mic_ampli_right, mic_ampli_back, mic_ampli_front, source_index);
#ifdef DEBUG_MAIN
        if(arg==ERROR_AUDIO){
        		chprintf((BaseSequentialStream *)&SD3, "Error in audioAnalyseDirection\n\r\n\r");
        		arg = 0;
        		updateAngle(arg);
        }
        else{
        		chprintf((BaseSequentialStream *)&SD3, "               Arg : 			%d\n\r", arg);
        		updateAngle(arg);
        }
#endif
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
