/*
 * main.c
 *
 *  Created on: Apr 1, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: Main file of this project. This is a penguin simulator,
 * 					where we will use the microphones to identify monofrequency
 * 					sources of sound, and find their direction. The user will
 * 					then be prompted using a serial bluetooth connection for
 * 					which source to go to (as if each source were a chick of
 * 					the penguin robot). Then, the robot will navigate towards
 * 					this source/chick... Unless dangerous killer whales are heard !
 *
 * 	Private functions prefix for this file (except main.c) : main_
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h> 					//needed for chibios chibios functionnality
#include <hal.h>					//needed also for base chibios functionnality, hardware abstraction layer specifically
#include <memory_protection.h>	//if unauthorised access, will throw into kernel panic and blink leds to tell user

//Project libraries, for sound processing, controlling the movements and serial communication via bluetooth
#include <audio_processing.h>
#include <travelController.h>
#include <comms.h>

#define DIR_SOURCE_MAX_TEXT_LENGTH	10
#define NUM_BASE_10					10
#define SOURCE_NOT_FOUND_THD			15

/*===========================================================================*/
/* Static, file wide defined variables                                       */
/*===========================================================================*/

/* @note robotMoving
 * Variable that defines if the robot is currently moving or not.
 * This is a file global variable because the main function depends on it,
 * but a callback from travelController (destReachedCB) will update it as well.
 */
static bool robotMoving = false;

/*===========================================================================*/
/* Internal functions definitions of main. Descriptions are above each 		*/
/* function code so as to have only one source of truth            			*/
/*===========================================================================*/
uint8_t main_scanSources(void);

void main_communicationUser(Destination *destination);

void main_moveTowardsTarget(Destination *destination);

/*===========================================================================*/
/* Public functions code & callbacks for outside main                       */
/*===========================================================================*/

/* @brief callback function to update the state in main of robotMoving
 * @note it is required by the travelController. It will be called when
 * 			the library the robots arrives at an obstacle.
 * 			The type and arguments follow the travCtrl_dirAngleCb_t type
 */
void destReachedCB(void)
{
	robotMoving = false;
}

/*===========================================================================*/
/* -----------------------  MAIN FUNCTION OF PROJECT -----------------------*/
/*===========================================================================*/

/*
 * @brief this is were the whole project begins, it actually never returns but
 * 			the type int is mandatory for convention purposes
 */
int main(void) {

	Destination destination;
	destination.index = UNINITIALIZED_INDEX;
	destination.freq = UNINITIALIZED_FREQ;
	destination.arg = 0;

	//Initialise chibios systems, hardware abstraction layer and memory protection
	halInit();
	chSysInit();
	mpu_init();

	//Start the serial communication over bluetooth
	comms_start();

	comms_printf(UART_PORT_STREAM, "Welcome to the penguin simulator!\n\r\n\r");
	comms_printf(UART_PORT_STREAM, "Our robot will try to beat penguins at their game,\n\r");
	comms_printf(UART_PORT_STREAM, "that is identifying sounds of their chicks and going\n\r");
	comms_printf(UART_PORT_STREAM, "to one of them amongst many.\n\r");
	comms_printf(UART_PORT_STREAM, "You will decide which onoe the penguins goes to...\n\r\n\r");
	comms_printf(UART_PORT_STREAM, "But be aware of killer whales... !\n\r");

	comms_printf(UART_PORT_STREAM, "\n\r\n\rThe simulation will now begin.\n\r\n\r");

	// Initialise motor controller. It does not move yet, as it waits for an angle
	travCtrl_init(destReachedCB);

	// Initialise audio module, which starts listening to mics and acquiring audio data
	audioP_init();

	/* Infinite main thread loop. */
	while (1){

		main_communicationUser(&destination); //Scan sources and ask the user to select one... except it there is a killer whale !

		main_moveTowardsTarget(&destination); //Move towards the selected source, until it is reached or not found

		chThdSleepMilliseconds(1000); //wait 1 second before restarting for final messages to finish sending to computer
	} //End of infinite main thread while loop
}

/*===========================================================================*/
/* Private functions							                                */
/*===========================================================================*/
/*
 * @brief find all monofrequency sound sources and retries until there are no errors
 *
 * @return number our sources which were found
 */
uint8_t main_scanSources(void)
{
	uint16_t nb_sources 								= 0;
	uint16_t audio_state 							= ERROR_AUDIO;
	Destination destination_scan[NB_SOURCES_MAX]		= {0};
	//TODOPING ask before starting scan, as sources have to be available
	//Scanning for sources until no ERROR is returned and number of sources is not equal to zero
	comms_printf(UART_PORT_STREAM, "Scanning for sources ...\n\r\n\r");
	while(audio_state==ERROR_AUDIO){ 				//we want to keep checking sources until some are found without errors

		audio_state = SUCCESS_AUDIO;
		nb_sources = audio_analyseSpectre();	//TODOPING maybe we should change, move, tell user to do something
		if(nb_sources==0 || nb_sources==ERROR_AUDIO){
			audio_state = ERROR_AUDIO;
			continue;
		}

		for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++) {
			destination_scan[source_counter].arg = audio_determineAngle(source_counter);
			destination_scan[source_counter].freq = audioGetSourceFreq(source_counter);
			if (destination_scan[source_counter].arg == ERROR_AUDIO || destination_scan[source_counter].freq == ERROR_AUDIO) {
				audio_state = ERROR_AUDIO;
				break;
			}
		}
	}

	//Printing the available sources and their frequencies
	comms_printf(UART_PORT_STREAM, "The following sources are available: \n\r");
	for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++) {	//We have checked errors above so we are sure none are present
		comms_printf(UART_PORT_STREAM,"Source %d :	 frequency =%u		angle =%d \n\r",
					 source_counter, audioConvertFreq(destination_scan[source_counter].freq), destination_scan[source_counter].arg);
	}

	return nb_sources;

}

void main_communicationUser(Destination *destination)
{
	//enter a new block (scope) to discard endTextReadPointer right after, and char array as well

	char readNumberText[DIR_SOURCE_MAX_TEXT_LENGTH];
	uint8_t readNumber;
	char *endTextReadPointer; //just to satisfy strtol arguments, but not useful for us
		//TODOPING around here if user presser R it re scans sources
	uint8_t numberOfSourcesDetected = main_scanSources();

	comms_printf(UART_PORT_STREAM, "Now please enter the number of the source you want our little penguin to go to\n\r");	//TODOPING check if it's withing number of souorcese
	comms_readf(UART_PORT_STREAM, readNumberText, DIR_SOURCE_MAX_TEXT_LENGTH);

	readNumber = (uint8_t) strtol(readNumberText, &endTextReadPointer, NUM_BASE_10);
	//TODOPING here check if it's within
#ifdef DEBUG_MAIN
	comms_printf(UART_PORT_STREAM, "You said %u ?\n\r\n\r", readNumber);
#endif

	destination->index = readNumber;	//TODOPING test for crazy inputs
	destination->freq = audioGetSourceFreq(destination->index);
}

void main_moveTowardsTarget(Destination *destination)
{
	uint16_t nb_sources				= 0;
	uint16_t audio_state				= ERROR_AUDIO;
	uint8_t 	sourceNotFoundCounter	= 0;

	robotMoving = true;

	while (robotMoving == true) {

		nb_sources = 0;
		audio_state = ERROR_AUDIO;
		sourceNotFoundCounter = 0;

		while(audio_state == ERROR_AUDIO){ //we want to keep checking sources until good data is ready
			audio_state = SUCCESS_AUDIO;

			nb_sources = audio_analyseSpectre();
			if(nb_sources==0 || nb_sources==ERROR_AUDIO){
				audio_state = ERROR_AUDIO;
				continue;
			}

			if(audio_updateDirection(destination) == SUCCESS_AUDIO){
				sourceNotFoundCounter = 0;
			}
			else if(sourceNotFoundCounter<SOURCE_NOT_FOUND_THD){
				sourceNotFoundCounter++;
				audio_state = ERROR_AUDIO;
				continue;
			}
			else{
				comms_printf(UART_PORT_STREAM, "The source you selected is not available anymore, please select a new one.\n\r");
				robotMoving = false;
				travCtrl_stopMoving();
				break;	//exit scanning while loop, should then exit robotMoving loop
			}


			destination->arg = audio_determineAngle(destination->index); //TODOPING rename arg into angle
			if(destination->arg == ERROR_AUDIO){
				audio_state = ERROR_AUDIO;
				continue;
			}

			/*TODOPING go away from the source*/
//			if(destination->arg >= 0){
//				destination->arg -= 180;
//			}
//			else{
//				destination->arg += 180;
//			}

			comms_printf(UART_PORT_STREAM, "nb_sources=%d	 index=%u 		freq=%u 		arg=%d\n\r",nb_sources, destination->index, audioConvertFreq(destination->freq), destination->arg); //DEBUG
		}

		if(robotMoving == true){	//in case the source is not found or destination was reached withing robotMoving loop
			travelCtrl_goToAngle(destination->arg);
		}
	} //end of while robotMoving

}

/*===========================================================================*/
/* Something to protect agains something...                                  */
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
