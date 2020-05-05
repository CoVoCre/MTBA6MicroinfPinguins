/*
 * main.c
 *
 *  Created on: Apr 1, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: Main file of this project. This is a penguin simulator,
 * 					where we will use the microphones to identify mono-frequency
 * 					sources of sound, and find their direction. The user will
 * 					then be prompted using a serial bluetooth connection for
 * 					which source to go to (as if each source were a chick of
 * 					the penguin robot). Then, the robot will navigate towards
 * 					this source/chick... Unless a dangerous killer whale is coming  !
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

/*===========================================================================*/
/* Definitions                                    						   */
/*===========================================================================*/

//Constants for text reading
#define DIR_SOURCE_MAX_TEXT_LENGTH	10
#define NUM_BASE_10					10

//Number constants
#define DEG180						180

/*===========================================================================*/
/* Static, file wide defined variables                                       */
/*===========================================================================*/

/* @note robotMoving
 * Variable that defines if the robot is currently moving or not.
 * This is a file variable because the main function depends on it,
 * but a callback from travelController (destReachedCB) will update it as well.
 */
static bool robotMoving		= false;

/* @note killerIsComing
 * Variable that defines if the robot is currently hunted by a killer whale or not.
 * This is a file variable because the main function depends on it,
 * but the thread ThdLed uses it as well
 */
static bool killerIsComing	= false;

/*===========================================================================*/
/* Internal functions definitions of main. 							 		*/
/*===========================================================================*/

/*
 * @brief	prints information in the beginning of the program
 */
void startPrintf(void);

/*
 * @brief 	scans sources and asks user which one to go to
 *
 *  @param[out] destination 			sets destination.angle and destination.freq of source selected by user
 */
void communicationUser(Destination *destination);

/*
 * @brief	calls audioP_analyseSources for the scanning of communicationUser and prints the available sources
 * @note		if killer whale is detected then escapesKiller is called
 *
 *  @param[out] destination_scan		puts angle and freq of available sources into destination_scan-array
 *
 * return	number of sources detected
 */
uint16_t detectSources(Destination *destination_scan);

/*
 * @brief 	supervises movement by scanning sources and updating direction angle as
 * 			the robot moves towards it, until the source is reached or not found anymore
 * @note		if killer whale is detected then escapesKiller is called
 *
 *  @param[out] destination 		the destination to go to, which will be updated as the robot moves
 *
 * return	AUDIOP__SUCCESS if destination is reached, AUDIOP__SOURCE_NOT_FOUND if source is not anymore found
 */
uint16_t moveTowardsDestination(Destination *destination);

/*
 * @brief	robot moves in the opposite direction than the killer whale sound is coming from
 */
void escapeKiller(void);

/*
 * @brief	printing the available sources and their frequencies.
 *
 *  @param[in] nb_sources		number of sources to print
 *  @param[in] destination_scan	array of sources containing for each their freq and angle
 */
void printSources(uint16_t nb_sources, Destination *destination_scan);

/*
 * @brief	sets body LED and moves backwards such that a new sources can be targeted
 * @note		is only called if the destination was reached
 */
void destinationReached(void);


/*===========================================================================*/
/* Public functions code & callbacks for outside main                       */
/*===========================================================================*/

/* @brief callback function to update the state in main of robotMoving
 * @note it is required by the travelController. It will be called by
 * 			travelController library when the robots arrives at an obstacle.
 * 			The type and arguments follow the travCtrl_dirAngleCb_t type
 */
void destReachedCB(void)
{
	robotMoving = false;
}


/*===========================================================================*/
/* Threads used in main                  								    */
/*===========================================================================*/

/* @brief thread created in fct. escapeKiller when escaping from killer whale
 * @note blinks LEDs as warning
 */
static THD_WORKING_AREA(waThdLed, 128);
static THD_FUNCTION(ThdLed, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(true){
    		while(killerIsComing){
    			chThdSleepMilliseconds(150);
    			palTogglePad(GPIOD, GPIOD_LED1);
			chThdSleepMilliseconds(150);
			palTogglePad(GPIOD, GPIOD_LED3);
			chThdSleepMilliseconds(150);
			palTogglePad(GPIOD, GPIOD_LED5);
			chThdSleepMilliseconds(150);
			palTogglePad(GPIOD, GPIOD_LED7);
    		}
    		palSetPad(GPIOD, GPIOD_LED1);
    		palSetPad(GPIOD, GPIOD_LED3);
    		palSetPad(GPIOD, GPIOD_LED5);
    		palSetPad(GPIOD, GPIOD_LED7);
    }
}


/*===========================================================================*/
/* -----------------------  MAIN FUNCTION OF PROJECT -----------------------*/
/*===========================================================================*/

int main(void)
{

	Destination destination;
	destination.freq = 	AUDIOP__UNINITIALIZED_FREQ;
	destination.angle = 	0;

	//Initialise chibios systems, hardware abstraction layer and memory protection
	halInit();
	chSysInit();
	mpu_init();

	//Start the serial communication over bluetooth
	comms_start();

	// Initialise motor controller. It does not move yet, as it waits for an angle
	travCtrl_init(destReachedCB);

	// Initialise audio module, which starts listening to mics and acquiring audio data
	audioP_init();

	//Thread for the LEDs when the killer whale is coming
	chThdCreateStatic(waThdLed, sizeof(waThdLed), NORMALPRIO, ThdLed, NULL);

	//prints information for starting
	startPrintf();

	/* Infinite main thread loop. */
	while (true){

		//Scan sources and ask the user to select one or escape from killer whale !
		communicationUser(&destination);

		//Move towards the selected source, until it is reached or not found anymore
		if(moveTowardsDestination(&destination) != AUDIOP__SOURCE_NOT_FOUND){
			destinationReached();
		}
	}
}



/*===========================================================================*/
/* Private functions							                                */
/*===========================================================================*/

void startPrintf(void)
{
	comms_printf("Welcome to the penguin-mother simulation!\n\r\n\r");
	comms_printf("Our robot-penguin-mother tries to feed their children,\n\r");
	comms_printf("but she needs our help to choose the child she should go to.\n\r");
	comms_printf( "The children can be distinguished by their crying at different frequencies.\n\r\n\r");
	comms_printf( "But be aware of killer whales... !\n\r");
	comms_printf( "\n\r\n\rThe simulation will now begin.\n\r\n\r");
}

void communicationUser(Destination *destination)
{

	char readNumberText[DIR_SOURCE_MAX_TEXT_LENGTH];
	uint8_t readNumber 	= AUDIOP__NB_SOURCES_MAX;					//AUDIOP__NB_SOURCES_MAX is not a valid number of sources
	char *endTextReadPointer; 										//pointer to store where strtol finishes reading text

	Destination destination_scan[AUDIOP__NB_SOURCES_MAX] = {0};		//for scanned sources
	bool keepAsking 										= true;		//this is the while control variable
	uint16_t nb_sources 									= 0;

	//Scanning for available sources (and killer whales 🐋)
	nb_sources = detectSources(destination_scan);

	//Keep asking until the the input is valid or r was pressed
	while(keepAsking == true){

		comms_printf( "\n\rPlease enter the number of the penguin you want to go to or enter 'r' to rescan penguins.\n\r");
		comms_readf( readNumberText, DIR_SOURCE_MAX_TEXT_LENGTH);

		//Entered number is verified
		if(readNumberText[0]!='r'){

			readNumber = (uint8_t) strtol(readNumberText, &endTextReadPointer, NUM_BASE_10);

			//No number was entered
			if(endTextReadPointer==readNumberText){
				comms_printf( "It seems what you just typed is not a number. Please try again !\n\r");
			}
			//Value entered is valid: between 0 (included) and nb_sources (excluded)
			else if(readNumber < nb_sources){
				keepAsking = false;
			}
			//The number wasn't a valid source or there were 0 sources available
			else{
				comms_printf( "It seems the number %u you just entered is not a valid source. Please try again !\n\r", readNumber);
			}
		}
		//r was pressed an sources are rescanned
		else{
			nb_sources = detectSources(destination_scan);
		}
	}

	comms_printf( "The robot will now go to penguin %u ...\n\r", readNumber);
	destination->freq = destination_scan[readNumber].freq;
	destination->angle = destination_scan[readNumber].angle;
}

uint16_t detectSources(Destination *destination_scan)
{
	uint16_t nb_sources 	= 0;

	nb_sources = audioP_analyseSources(destination_scan);
	while(nb_sources==AUDIOP__KILLER_WHALE_DETECTED){
		escapeKiller();
		travCtrl_stopMoving();
		nb_sources = audioP_analyseSources(destination_scan);
	}

	printSources(nb_sources, destination_scan);

	return nb_sources;
}

uint16_t moveTowardsDestination(Destination *destination)
{
	uint16_t analyseDestination		= 0;

	/*Now we set robotMoving file variable to true, it will either be set to false here if
	 * we cannot find the destination source anymore, or by the destReachedCB function
	 * called from travelController when the robot encounters an obstacle. */
	robotMoving = true;

	while (robotMoving == true) {

		//Scans sound for destination source and updates angle
		analyseDestination = audioP_analyseDestination(destination);

		//Check if the selected source is still available or a killer whale was detected
		if(analyseDestination==AUDIOP__SOURCE_NOT_FOUND){
			comms_printf( "\n\rThe source is not anymore available, please select a new one.\n\r");
			robotMoving = false;
			travCtrl_stopMoving();
			return AUDIOP__SOURCE_NOT_FOUND;
		}
		else if(analyseDestination==AUDIOP__KILLER_WHALE_DETECTED){
			escapeKiller();
			robotMoving = true;
		}
		else{
			travelCtrl_goToAngle(destination->angle);
		}
	}

	return AUDIOP__SUCCESS;
}

void escapeKiller(void)
{
	Destination killer;
	killer.freq =	AUDIOP__UNINITIALIZED_FREQ;
	killer.angle = 	0;

	killerIsComing =true;
	while(killerIsComing){
		if(audioP_analyseKiller(&killer) == AUDIOP__SOURCE_NOT_FOUND){
			killerIsComing = false;
		}
		else{
			if(killer.angle >= 0){								//Go in opposite direction than the sound is coming from
				travelCtrl_goToAngle(killer.angle-DEG180);
			}
			else{
				travelCtrl_goToAngle(killer.angle+DEG180);
			}
		}
	}

	comms_printf( "We escaped from the killer whale!\n\r");
}

void printSources(uint16_t nb_sources, Destination *destination_scan)
{
	//We are sure there are no more errors or killer whales inside destination_scan and, so we do not check for errors anymore
	comms_printf( "The following sources are available: \n\r");
	if(nb_sources==0){
		comms_printf("    ...No sources we found...\n\r");

	}
	else{
		for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++) {
			comms_printf("Source %d :	 frequency =%u		angle =%d \n\r", source_counter,
					audioP_convertFreq(destination_scan[source_counter].freq), destination_scan[source_counter].angle);
		}
	}
}

void destinationReached(void)
{
	comms_printf("\n\r\n\r Final destination reached! \n\r\n\r\n\r");
	palSetPad(GPIOB, GPIOB_LED_BODY);
	chThdSleepMilliseconds(1500);

	travCtrl_moveBackwards();			//moves backwards in case robot wants to go to another source
	chThdSleepMilliseconds(3000);

	travCtrl_stopMoving();
	palClearPad(GPIOB, GPIOB_LED_BODY);
}



/*===========================================================================*/
/* Code to protect against smashing */
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
