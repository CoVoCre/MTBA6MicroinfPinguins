#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arm_math.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>

#include <main.h>
#include <audio_processing.h>
#include <fft.h>
#include <travelController.h>
#include <comms.h>

/*Enable for Debugging main*/
#define DEBUG_MAIN

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

#define NB_BYTE_PER_CMPX_VAL			2
#define DIR_SOURCE_MAX_TEXT_LENGTH	10
#define NUM_BASE_10					10

bool robotMoving = false;

/* Time measuring :
 * systime_t time = chVTGetSystemTime();
 *  printf("It took %d", ST2MS(time-chVTGetSystemTime() ));
 *
 */
void destReachedCB(void) {
#ifdef DEBUG_MAIN
	chprintf(UART_PORT_STREAM,
			"---------------------------------------------------------------\n\r");
	chprintf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	chprintf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	chprintf(UART_PORT_STREAM,
			"WARNING test_destReachedCB was called and waiting 1second\n\r");
	chprintf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	chprintf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	chprintf(UART_PORT_STREAM,
			"---------------------------------------------------------------\n\r");
#endif // DEBUG_MAIN
	robotMoving = false;
}

int main(void) {
	//send_tab is used to save the state of the buffer to send (double buffering)
	//to avoid modifications of the buffer while sending it
	static float mic_data_right[NB_BYTE_PER_CMPX_VAL * FFT_SIZE];
	static float mic_data_left[NB_BYTE_PER_CMPX_VAL * FFT_SIZE];
	static float mic_data_front[NB_BYTE_PER_CMPX_VAL * FFT_SIZE];
	static float mic_data_back[NB_BYTE_PER_CMPX_VAL * FFT_SIZE];
	static float mic_ampli_right[FFT_SIZE];
	static float mic_ampli_left[FFT_SIZE];
	static float mic_ampli_front[FFT_SIZE];
	static float mic_ampli_back[FFT_SIZE];

	Destination destination;
	destination.index = UNINITIALIZED_INDEX;
	destination.freq = UNINITIALIZED_FREQ;
	destination.arg = 0;

	int16_t audio_peak = 0;
	uint16_t nb_sources = 0;

	halInit();
	chSysInit();
	mpu_init();

	comms_start();

#ifdef DEBUG_MAIN
	comms_printf(UART_PORT_STREAM, "Starting main !\n\r");
#endif //DEBUG_MAIN

	//init motor controller, does not move yet, wait for angle
	travCtrl_init(destReachedCB);

	//init audio module, start listening to mics and acquiring audio
	audioP_init();

	/* Infinite main thread loop. */
	while (1) {
		//TODOPING break down code to smaller sub functions, here could be till audioCalculateFFT

		//Waits until enough sound samples are collected
		wait_send_to_computer(); //TODOPING rename function or just reorganize code

		//Copy buffer to avoid conflicts
		arm_copy_f32(get_audio_buffer_ptr(LEFT_CMPLX_INPUT), mic_data_left,
				NB_BYTE_PER_CMPX_VAL * FFT_SIZE);
		arm_copy_f32(get_audio_buffer_ptr(RIGHT_CMPLX_INPUT), mic_data_right,
				NB_BYTE_PER_CMPX_VAL * FFT_SIZE);
		arm_copy_f32(get_audio_buffer_ptr(FRONT_CMPLX_INPUT), mic_data_front,
				NB_BYTE_PER_CMPX_VAL * FFT_SIZE);
		arm_copy_f32(get_audio_buffer_ptr(BACK_CMPLX_INPUT), mic_data_back,
				NB_BYTE_PER_CMPX_VAL * FFT_SIZE);

		//Calculate FFT of sound signal
		audioCalculateFFT(mic_data_left, mic_data_right, mic_data_back,
				mic_data_front, mic_ampli_left, mic_ampli_right, mic_ampli_back,
				mic_ampli_front);

		/*Find peak intensity sources and sort them according to frequency,
		 * returns angle of destination source but here it is not needed
		 * NB for the first time, destination is not important, we have it uninitialized which works out */
		audioPeak(mic_ampli_left, &destination);

		//get how many sources were detected, in order to present them to user for choosing
		nb_sources = audioGetNbSources();
		if (nb_sources == ERROR_AUDIO) {
			chprintf((BaseSequentialStream *) &SD3,
					"There was an error, please restart robot \n\r\n\r");
			/*TODOPING, for all of audio there should only be one error returned which causes total stop
			 * main should not know of specific errors, except if it can do something about them, ex :
			 * "audio not initialized, you need to call ..."
			 */
			while (1) {
			} //TODOPING stop everything else, create function for this
		}
		if (nb_sources > 0) { //if no sources detected we will just try again next time there is audio available //TODOPING is this behavior ok ?
			comms_printf(UART_PORT_STREAM, "There are sources available !\n\r");
			for (uint8_t i = 0; i < nb_sources; i++) {
				int16_t source_angle = audioDetermineAngle(mic_data_left,
						mic_data_right, mic_data_back, mic_data_front, i);
				uint16_t source_freq = audioGetSourceFreq(i);
				if (source_angle == ERROR_AUDIO || source_freq == ERROR_AUDIO) {
					chprintf((BaseSequentialStream *) &SD3,
							"There was an error, please restart robot \n\r\n\r");
					while (1) {
					} //TODOPING stop everything else
				}
				comms_printf(UART_PORT_STREAM,
						"Source %d has angle =%d and frequency =%u\n\r", i,
						source_angle, source_freq);
			}

			comms_printf(UART_PORT_STREAM,
					"Now please enter the number of the source you want our little penguin to go to\n\r");
			char readNumberText[DIR_SOURCE_MAX_TEXT_LENGTH];
			uint8_t readNumber;
			comms_readf(UART_PORT_STREAM, readNumberText,
					DIR_SOURCE_MAX_TEXT_LENGTH);
			{//enter a new block (scope) to discard endTextReadPointer right after
				char *endTextReadPointer; //just to satisfy strtol arguments, but not useful for us
				readNumber = (uint8_t) strtol(readNumberText,
						&endTextReadPointer, NUM_BASE_10);
#ifdef DEBUG_MAIN
				chprintf((BaseSequentialStream *) &SD3, "You said %u ?\n\r\n\r",
						readNumber);
#endif
			}

			destination.index = readNumber;
			destination.freq = audioGetSourceFreq(destination.index);

			robotMoving = true;

			travelCtrl_goToAngle(audioDetermineAngle(mic_data_left,mic_data_right, mic_data_back, mic_data_front, destination.index));

			while (robotMoving == true) {

				audio_peak = audioPeak(mic_ampli_left, &destination);
				if (audio_peak == ERROR_AUDIO) {
#ifdef DEBUG_MAIN
					chprintf((BaseSequentialStream *) &SD3,
							"main:	Error in audioPeak\n\r\n\r");
#endif
				} else if (audio_peak == ERROR_AUDIO_SOURCE_NOT_FOUND) {
#ifdef DEBUG_MAIN
					chprintf((BaseSequentialStream *) &SD3,
							"main:	Error source not found ! \n\r\n\r");
#endif
				} else {
					if (destination.index == UNINITIALIZED_INDEX) {
#ifdef DEBUG_MAIN
						chprintf((BaseSequentialStream *) &SD3,
								"main:	UNINITIALIZED_INDEX\n\r\n\r");
#endif
					} else {
						destination.arg = audioDetermineAngle(mic_data_left,
								mic_data_right, mic_data_back, mic_data_front,
								destination.index);
						if (destination.arg == ERROR_AUDIO) {
#ifdef DEBUG_MAIN
							chprintf((BaseSequentialStream *) &SD3,
									"main:	Error in audioAnalyseDirection\n\r\n\r");
#endif
							destination.arg = 0;
							travelCtrl_goToAngle(destination.arg);
						} else {
#ifdef DEBUG_MAIN
							chprintf((BaseSequentialStream *) &SD3,
									"main:	Source %d :		Freq %d	:		arg  = %d\n\r",
									destination.index,
									audioConvertFreq(destination.freq),
									destination.arg);
#endif
							travelCtrl_goToAngle(destination.arg);
						}
					}
				}
			}
		} // end of if(nb_sources>0)
	} //End of infinite main thread loop
}



/*===========================================================================*/
/* Something to protect agains something...                                  */
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
