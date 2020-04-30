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

//#define NB_BYTE_PER_CMPX_VAL			2
#define DIR_SOURCE_MAX_TEXT_LENGTH	10
#define NUM_BASE_10					10
#define SOURCE_NOT_FOUNF_THD			20


//travCtrl_dirAngleCb_t updateAngle;	//TODOPING not a callback anymore
//bool robotMoving = false;
bool robotMoving = false;

/* Time measuring :
 * systime_t time = chVTGetSystemTime();
 *  printf("It took %d", ST2MS(time-chVTGetSystemTime() ));
 *
 */

void destReachedCB(void) {
	robotMoving = false;
#ifdef DEBUG_MAIN
	comms_printf(UART_PORT_STREAM,
			"---------------------------------------------------------------\n\r");
	comms_printf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	comms_printf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	comms_printf(UART_PORT_STREAM,
			"WARNING test_destReachedCB was called \n\r");
	comms_printf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	comms_printf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	comms_printf(UART_PORT_STREAM,
			"---------------------------------------------------------------\n\r");
#endif // DEBUG_MAIN
}

void main_unhandledError(void){
	comms_printf(UART_PORT_STREAM,
				"---------------------------------------------------------------\n\r");
	comms_printf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	comms_printf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	comms_printf(UART_PORT_STREAM,
			"CRITICAL ERROR please reset robot\n\r");
	comms_printf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	comms_printf(UART_PORT_STREAM,
			"-                                                             -\n\r");
	comms_printf(UART_PORT_STREAM,
			"---------------------------------------------------------------\n\r");
	while(1){}
}

int main(void) {
//	//arrays used to save the state of the mic audio buffer (double buffering)
//	//to avoid modifications of the buffer while analysing it
//	static float mic_data_right[NB_BYTE_PER_CMPX_VAL * FFT_SIZE];
//	static float mic_data_left[NB_BYTE_PER_CMPX_VAL * FFT_SIZE];
//	static float mic_data_front[NB_BYTE_PER_CMPX_VAL * FFT_SIZE];
//	static float mic_data_back[NB_BYTE_PER_CMPX_VAL * FFT_SIZE];
//	static float mic_ampli_right[FFT_SIZE];
//	static float mic_ampli_left[FFT_SIZE];
//	static float mic_ampli_front[FFT_SIZE];
//	static float mic_ampli_back[FFT_SIZE];

	Destination destination;
	destination.index = UNINITIALIZED_INDEX;
	destination.freq = UNINITIALIZED_FREQ;
	destination.arg = 0;

	uint16_t nb_sources = 0;
	int16_t audio_state = SUCCESS_AUDIO;

	halInit();
	chSysInit();
	mpu_init();

	comms_start();

	comms_printf(UART_PORT_STREAM, "Welcome to the penguin simulator!\n\r");
	comms_printf(UART_PORT_STREAM, "Our robot will try to beat penguins at their game, p\n\r");
	comms_printf(UART_PORT_STREAM, "that is identifying sounds of their children and going\n\r");
	comms_printf(UART_PORT_STREAM, "to one of them amongst many.\n\r");
	comms_printf(UART_PORT_STREAM, "\n\rThe simulation will begin shortly...\n\r\n\r");

	//init motor controller, does not move yet, wait for angle
	travCtrl_init(destReachedCB);

	//init audio module, start listening to mics and acquiring audio data
	audioP_init();

	/* Infinite main thread loop. */
	while (1){

		//TODOPING break down code to smaller sub functions, here could be till audioCalculateFFT

		//Scanning for sources until no ERROR is returned and number of sources is not equal to zero
		comms_printf(UART_PORT_STREAM, "Will now scan sources for display...\n\r\n\r");
		nb_sources=0;
		audio_state = ERROR_AUDIO;
		while(audio_state==ERROR_AUDIO){ 							//we want to keep checking sources until some are found
			comms_printf(UART_PORT_STREAM, "main: inside first while\n\r\n\r");

			audio_state = SUCCESS_AUDIO;
			nb_sources = audio_analyseSpectre();	//TODOPING maybe we should change, move, tell user to do something
			if(nb_sources==0 || nb_sources==ERROR_AUDIO){
				audio_state = ERROR_AUDIO;
				continue;
			}
			comms_printf(UART_PORT_STREAM, "main: after analyseSpectre\n\r\n\r");

			for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++) {
				if (audio_determineAngle(source_counter) == ERROR_AUDIO || audioGetSourceFreq(source_counter) == ERROR_AUDIO) {
					audio_state = ERROR_AUDIO;
					continue;
				}
			}
			comms_printf(UART_PORT_STREAM, "main: after determineAngle\n\r\n\r");
		}

		//Printing the available sources and their frequencies
		comms_printf(UART_PORT_STREAM, "The following sources are available: \n\r");
		for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++) {
			destination.arg = audio_determineAngle(source_counter); 								//We have checked errors above so we are sure none are present //audioDetermineAngle(mic_data_left, mic_data_right, mic_data_back, mic_data_front, current_source);
			destination.freq = audioGetSourceFreq(source_counter);									//TODOPING find better way than calculating this two times
			comms_printf(UART_PORT_STREAM,"Source %d :	 angle =%d 	and		 frequency =%u\n\r", source_counter, destination.arg, destination.freq);
		}


		{//enter a new block (scope) to discard endTextReadPointer right after, and char array as well
			comms_printf(UART_PORT_STREAM, "Now please enter the number of the source you want our little penguin to go to\n\r");
			char readNumberText[DIR_SOURCE_MAX_TEXT_LENGTH];
			uint8_t readNumber;
			comms_readf(UART_PORT_STREAM, readNumberText, DIR_SOURCE_MAX_TEXT_LENGTH);
			char *endTextReadPointer; //just to satisfy strtol arguments, but not useful for us
			readNumber = (uint8_t) strtol(readNumberText, &endTextReadPointer, NUM_BASE_10);
#ifdef DEBUG_MAIN
			chprintf((BaseSequentialStream *) &SD3, "You said %u ?\n\r\n\r", readNumber);
#endif
			destination.index = readNumber;	//TODOPING test for crazy inputs
			destination.freq = audioGetSourceFreq(destination.index);
		}

		robotMoving = true;

		while (robotMoving == true) {
			nb_sources=0;
			audio_state = ERROR_AUDIO;
			uint8_t 	sourceNotFoundCounter=0;

			while(audio_state == ERROR_AUDIO){ //we want to keep checking sources until good data is ready

				nb_sources = audio_analyseSpectre();
				if(nb_sources==0 || nb_sources==ERROR_AUDIO){
					audio_state = ERROR_AUDIO;
					continue;
				}

				if(audio_updateDirection(&destination) == SUCCESS_AUDIO){
					sourceNotFoundCounter = 0;
				}
				else if(sourceNotFoundCounter<SOURCE_NOT_FOUNF_THD){
					sourceNotFoundCounter++;
					audio_state = ERROR_AUDIO;
					continue;
				}
				else{
					comms_printf(UART_PORT_STREAM, "The source you selected is not available anymore, please select a new one.\n\r");
					robotMoving = false;
					travCtrl_stopMoving();
					break;	//exit scanning while loop, shuld then exit robotMoving loop
				}

				destination.arg = audio_determineAngle(destination.index);
				if(destination.arg == ERROR_AUDIO){
					audio_state = ERROR_AUDIO;
					continue;
				}

				//TODOPING Which arg should we give to the motors?
//				if(tempAudioDirectionAngle==ERROR_AUDIO){
//					travelCtrl_goToAngle(0);
//				}

				//comms_printf(UART_PORT_STREAM, "robotMoving:scanning : nb_sources=%d, tempIndexErrorUpdate=%d destination.index=%u destination.freq=%u tempAudioDirectionAngle=%d\n\r",nb_sources, tempIndexErrorUpdate, destination.index, destination.freq, tempAudioDirectionAngle); //DEBUG
			}

			if(robotMoving == true){	//in case the source is not found or destination was reached withing robotMoving loop
				travelCtrl_goToAngle(destination.arg);
				//comms_printf(UART_PORT_STREAM, "audioP_determineSrcAngle was called with angle=%d\n\r",tempAudioDirectionAngle); //DEBUG
			}
		} //end of while robotMoving

	chThdSleepMilliseconds(1000); //wait 1 second before restarting for final messages to finish sending to computer
	} //End of infinite main thread while loop
}

/*===========================================================================*/
/* Something to protect agains something...                                  */
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
