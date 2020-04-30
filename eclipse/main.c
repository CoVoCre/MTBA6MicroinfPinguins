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

//travCtrl_dirAngleCb_t updateAngle;
//bool robotMoving = false;

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

/* Time testing :
 * systime_t time = chVTGetSystemTime();
 *  printf("It took %d", ST2MS(time-chVTGetSystemTime() ));
 *
 */
//void destReachedCB(void){
//#ifdef DEBUG_MAIN
//	chprintf(UART_PORT_STREAM,"---------------------------------------------------------------\n\r");
//	chprintf(UART_PORT_STREAM,"-                                                             -\n\r");
//	chprintf(UART_PORT_STREAM,"-                                                             -\n\r");
//	chprintf(UART_PORT_STREAM,"WARNING test_destReachedCB was called and waiting 1second\n\r");
//	chprintf(UART_PORT_STREAM,"-                                                             -\n\r");
//	chprintf(UART_PORT_STREAM,"-                                                             -\n\r");
//	chprintf(UART_PORT_STREAM,"---------------------------------------------------------------\n\r");
//#endif // DEBUG_MAIN
//	chprintf(UART_PORT_STREAM,"Reached destination !!!\n\r");
//	robotMoving=false;
//}

int main(void)
{
	halInit(); //
	chSysInit();
	mpu_init();
	//timer12_start();

	comms_start();
	chprintf(UART_PORT_STREAM,"Starting main !\n\r");

	//updateAngle = travCtrl_init(destReachedCB);

	//TESTPING test travelController functions!
	//travCtrl_testAll();

	comms_printf(UART_PORT_STREAM, "In MAIN \n\r");
	comms_printf(UART_PORT_STREAM, "We will now ask for text. Enter anything then press enter\n\r");

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

    Destination destination;
    int16_t audio_peak			= 0;
    uint16_t nb_sources			= 0;

    destination.index		= UNINITIALIZED_INDEX;
    destination.freq			= UNINITIALIZED_FREQ;
    destination.arg			= 0;



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

    	        /*Calculating FFT and its amplitude*/
    	        audioCalculateFFT(mic_data_left, mic_data_right, mic_data_back, mic_data_front, mic_ampli_left, mic_ampli_right, mic_ampli_back, mic_ampli_front);
    	        destination.arg=audioPeak(mic_ampli_left, &destination);
    }
}

//    		audioPeak(mic_ampli_left, &destination);
//
//    		nb_sources = audioGetNbSources();
//    		if(nb_sources==ERROR_AUDIO){
//        		chprintf((BaseSequentialStream *)&SD3, "There was an error, please restart robot \n\r\n\r");
//        		while(1){} //TODOPING stop everything else
//    		}
//    		if(nb_sources>0){
//		comms_printf("There are sources available !\n\r");
//
//		for(uint8_t i=0; i<nb_sources; i++){
//			int16_t source_angle = audioDetermineAngle(mic_data_left, mic_data_right, mic_data_back, mic_data_front, i);
//			uint16_t source_freq = audioGetSourceFreq(i);
//			if(source_angle==ERROR_AUDIO || source_freq==ERROR_AUDIO){
//				chprintf((BaseSequentialStream *)&SD3, "There was an error, please restart robot \n\r\n\r");
//				while(1){} //TODOPING stop everything else
//			}
//    			comms_printf(UART_PORT_STREAM, "Source %d has angle =%d and frequency =%u\n\r", i,source_angle, source_freq);
//		}
//
//		comms_printf(UART_PORT_STREAM, "Now please enter the number of the source you want our little penguin to go to\n\r");
//		uint8_t readNumberText[SOURCE_MAX_TEXT_LENGTH];
//		uint8_t readNumber;
//		comms_readf(UART_PORT_STREAM, readNumberText, SOURCE_MAX_TEXT_LENGTH);
//		readNumber = (uint8_t) strtol(readNumberText, readNumberText+SOURCE_MAX_TEXT_LENGTH-1,NUM_BASE_10);
//#ifdef DEBUG_MAIN
//        	chprintf((BaseSequentialStream *)&SD3, "You said %d ?\n\r\n\r", readNumber);
//#endif
//        	destination.index = readNumber;
//        	destination.freq = audioGetSourceFreq(destination.index);
//
//        	robotMoving=true;
//        	travCtrl_startStop(robotMoving);
//        	while(robotMoving==true){
//
//			/*Testing two sources*/
//
//			audio_peak = audioPeak(mic_ampli_left, &destination);
//			if(audio_peak==ERROR_AUDIO){
//	#ifdef DEBUG_MAIN
//					chprintf((BaseSequentialStream *)&SD3, "main:	Error in audioPeak\n\r\n\r");
//	#endif
//			}
//			else if(audio_peak==ERROR_AUDIO_SOURCE_NOT_FOUND){
//	#ifdef DEBUG_MAIN
//				chprintf((BaseSequentialStream *)&SD3, "main:	Error source not found ! \n\r\n\r");
//	#endif
//			}
//			else{
//					if(destination.index==UNINITIALIZED_INDEX){
//	#ifdef DEBUG_MAIN
//						chprintf((BaseSequentialStream *)&SD3, "main:	UNINITIALIZED_INDEX\n\r\n\r");
//	#endif
//					}
//					else{
//						destination.arg = audioDetermineAngle(mic_data_left, mic_data_right, mic_data_back, mic_data_front, destination.index);
//						if(destination.arg==ERROR_AUDIO){
//	#ifdef DEBUG_MAIN
//							chprintf((BaseSequentialStream *)&SD3, "main:	Error in audioAnalyseDirection\n\r\n\r");
//	#endif
//							destination.arg = 0;
//							updateAngle(destination.arg);
//					}
//						else{
//	#ifdef DEBUG_MAIN
//							chprintf((BaseSequentialStream *)&SD3, "main:	Source %d :		Freq %d	:		arg  = %d\n\r", destination.index, audioConvertFreq(destination.freq), destination.arg);
//	#endif
//							updateAngle(destination.arg);
//						}
//					}
//
//			}
//        	}
 //   }
//}

/*===========================================================================*/
/* Something to protect agains something...                                  */
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
