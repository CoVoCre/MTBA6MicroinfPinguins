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
/* Definitions                                       */
/*===========================================================================*/

#define DIR_SOURCE_MAX_TEXT_LENGTH	10
#define NUM_BASE_10					10
#define SOURCE_NOT_FOUND_THD			15

/* @note NOT_A_VALID_SOURCE
 * There are not over 100 sources, so we use 100 as error
 * code for when the user asks us to go to an unexisting source*/
#define NOT_A_VALID_SOURCE		100

/*===========================================================================*/
/* Static, file wide defined variables                                       */
/*===========================================================================*/

/* @note robotMoving
 * Variable that defines if the robot is currently moving or not.
 * This is a file variable because the main function depends on it,
 * but a callback from travelController (destReachedCB) will update it as well.
 */
static bool robotMoving = false;

/*===========================================================================*/
/* Internal functions definitions of main. Descriptions are above each 		*/
/* function code so as to have only one source of truth            			*/
/*===========================================================================*/

//uint8_t getSourcesAndCheckKillerWhales(Destination *destination_scan, Destination *destination);

void communicationUser(Destination *destination);

uint16_t moveTowardsTarget(Destination *destination);

void escapeKiller(void);

void printSources(uint16_t nb_sources, Destination *destination_scan);

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
/* Threads of the main                   								    */
/*===========================================================================*/
/* @brief thread created in fct. scapeKiller when escaping from killer whale
 * @note blinks LEDs as warning
 */
static THD_WORKING_AREA(waThdLed, 128);
static THD_FUNCTION(ThdLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //chThdSleepMilliseconds(400);			//Waits in case the thread is directly again stopped TODOPING

    while(!chThdShouldTerminateX()){
		palTogglePad(GPIOD, GPIOD_LED1);
		chThdSleepMilliseconds(150);
		palTogglePad(GPIOD, GPIOD_LED3);
		chThdSleepMilliseconds(150);
		palTogglePad(GPIOD, GPIOD_LED5);
		chThdSleepMilliseconds(150);
		palTogglePad(GPIOD, GPIOD_LED7);
        chThdSleepMilliseconds(150);
    }
	palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);
}


/*===========================================================================*/
/* -----------------------  MAIN FUNCTION OF PROJECT -----------------------*/
/*===========================================================================*/

/*
 * @brief this is were the whole project begins
 * @note it actually never returns but the type int is mandatory
 * 			for convention purposes
 */
int main(void) {

	Destination destination;
	destination.index = 	AUDIOP__UNINITIALIZED_INDEX;
	destination.freq = 	AUDIOP__UNINITIALIZED_FREQ;
	destination.angle = 	0;

	//Initialise chibios systems, hardware abstraction layer and memory protection
	halInit();
	chSysInit();
	mpu_init();

	//Start the serial communication over bluetooth
	comms_start();

	comms_printf("Welcome to the penguin-mother simulation!\n\r\n\r");
	comms_printf("Our robot-penguin-mother tries to feed their children,\n\r");
	comms_printf("but she needs our help to choose the child she should go to.\n\r");
	comms_printf( "The children can be distinguished by their crying at different frequencies.\n\r\n\r");
	comms_printf( "But be aware of killer whales... !\n\r");

	comms_printf( "\n\r\n\rThe simulation will now begin.\n\r\n\r");

	// Initialise motor controller. It does not move yet, as it waits for an angle
	travCtrl_init(destReachedCB);

	// Initialise audio module, which starts listening to mics and acquiring audio data
	audioP_init();

	/* Infinite main thread loop. */
	while (1){

		communicationUser(&destination); //Scan sources and ask the user to select one... except if there is a killer whale !

		if(moveTowardsTarget(&destination) != ERROR_AUDIO_SOURCE_NOT_FOUND){ //Move towards the selected source, until it is reached or not found anymore
			destinationReached();
		}

	} //End of infinite main thread while loop
}

/*===========================================================================*/
/* Private functions							                                */
/*===========================================================================*/

/*
 * @brief scans sources (and killer whales !) and asks user which one to go to
 *
 * @param[out] destination will set destination.index and destination.freq of source selected by user
 */
void communicationUser(Destination *destination) {
	/* @note readNumberText
	 * We will read text from serial communication, and try to convert it to a
	 * number or check if it's a special command. Therefore we need an array of
	 * chars (equivalent of a string).  */
	char readNumberText[DIR_SOURCE_MAX_TEXT_LENGTH];
	uint8_t readNumber = NOT_A_VALID_SOURCE;	//To store the converted number if it's no error
	char *endTextReadPointer; //pointer to store where strtol finishes reading text

	Destination destination_scan[AUDIOP__NB_SOURCES_MAX] = {0};		//we store all found sources properties here
	bool keepAsking 										= true;											//this is the while control variable, to ask and re ask until a source is selected
	uint16_t nb_sources 									= 0;

	/* Scans sources until there are no errors and no killer whales 🐋,
	 * and returns how many were found at that point */
						//TODOPING put it in a function because reused afterwards!
	nb_sources = audio_analyseSources(destination_scan);
	if(nb_sources==KILLER_WHALE_DETECTED){
		escapeKiller();
	}

	printSources(nb_sources, destination_scan);

	//we ask the user for a number or r (rescan), and keep asking until the the input is valid
	while(keepAsking == true){

		comms_printf( "\n\rPlease enter the number of the penguin you want to go to or enter 'r' to rescan penguins.\n\r");
		comms_readf( readNumberText, DIR_SOURCE_MAX_TEXT_LENGTH);

		//If r isn't entered, we try to interpret the number entered
		if(readNumberText[0]!='r'){

			readNumber = (uint8_t) strtol(readNumberText, &endTextReadPointer, NUM_BASE_10);

			/* Three cases are possible : the value entered was not a valid number, or it was and was
			 * one of the sources, or it wasn't one of the sources (also if no source is available and
			 * user didn't press r) */
			if(endTextReadPointer==readNumberText){//strtol didn't find a number as it stopped reading at the beginning
				comms_printf( "It seems what you just typed is not a number. Please try again !\n\r");
			}
			//We check if the value entered is actually a source, between 0 (excluded) and nb_sources (excluded)
			else if(readNumber < nb_sources){
				keepAsking = false;	//Here we will actually have a good source
			}
			else{	//the number wasn't a valid source or there were 0 sources available and user didn't press r
				comms_printf( "It seems the number %u you just entered is not a valid source. Please try again !\n\r", readNumber);
			}
		}
		else{ //otherwise (user typed r) we rescan
			nb_sources = audio_analyseSources(destination_scan);
			if(nb_sources==KILLER_WHALE_DETECTED){
				escapeKiller();
			}

			printSources(nb_sources, destination_scan);
		}
	}

	comms_printf( "The robot will now go to penguin %u ...\n\r", readNumber);
	destination->index = readNumber;
	destination->freq = destination_scan[destination->index].freq;
	destination->angle = destination_scan[destination->index].angle;
}

/*
 * @brief supervises movement by scanning sources and updating direction angle as
 * 			the robot moves towards it, until it is reached or not found anymore
 * @note when scanning sources, if there is a killer whale it will instantly start to move away from it
 *
 * @param[out] destination 	the destination to go to, which will be updated as the robot moves,
 * 								when the frequency changes a little bit and the index might change
 * 								as other sources change. The angle is constantly recalculated.
 * 								Not intended to be used as output, but still this function
 * 								modifies this variable.
 */
uint16_t moveTowardsTarget(Destination *destination)
{
	uint16_t analyseDestination		= 0;

	/*Now we set robotMoving file variable to true, it will either be set to false here if
	 * we cannot find the destination source anymore, or by the destReachedCB function
	 * called from travelController when the robot encounters an obstacle. */
	robotMoving = true;

	while (robotMoving == true) {

		/*scanSources returns only when there are no killer whales
		 * (if killer whale is present, it will run away from it). */
		analyseDestination = audio_analyseDestination(destination);

		//Now we check if the selected source is still available
		if(analyseDestination==ERROR_AUDIO_SOURCE_NOT_FOUND){
			comms_printf( "\n\rThe source is not anymore available, please select a new one.\n\r");
			robotMoving = false;
			travCtrl_stopMoving();
			return ERROR_AUDIO_SOURCE_NOT_FOUND;
		}
		else if(analyseDestination==KILLER_WHALE_DETECTED){
			escapeKiller();
			robotMoving = true;
		}
		else{
			travelCtrl_goToAngle(destination->angle);
		}
	} //end of while robotMoving

	return SUCCESS_AUDIO;
}

void escapeKiller(void)
{
	Destination killer;
	killer.index = 	AUDIOP__UNINITIALIZED_INDEX;
	killer.freq =	AUDIOP__UNINITIALIZED_FREQ;
	killer.angle = 	0;

	bool killerIsComing	= true;

	thread_t *blink = chThdCreateStatic(waThdLed, sizeof(waThdLed), NORMALPRIO, ThdLed, NULL);

	while(killerIsComing){
		if(audio_analyseKiller(&killer) == ERROR_AUDIO_SOURCE_NOT_FOUND){
			killerIsComing = false;
		}
		else{
			if(killer.angle >= 0){
				travelCtrl_goToAngle(killer.angle-180);
			}
			else{
				travelCtrl_goToAngle(killer.angle+180);
			}
		}
	}

	chThdTerminate(blink);
	comms_printf( "We escaped from the killer whale!\n\r");
}

void printSources(uint16_t nb_sources, Destination *destination_scan){
	/* Printing the available sources and their frequencies.
	 * When we get here we are sure there are no more errors and killer whales, so we do not check anymore */
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
	palSetPad(GPIOB, GPIOB_LED_BODY);;
	chThdSleepMilliseconds(1000); //wait 1 second before restarting for final messages to finish sending to computer
	travCtrl_moveBackwards();
	chThdSleepMilliseconds(3000);
	travCtrl_stopMoving();
	palClearPad(GPIOB, GPIOB_LED_BODY);
}

/*
 * @brief get all monofrequency sound sources and retries until there are no killer whales 🐋
 * @note if there is a killer whale, it will make the robot penguin run away from it
 *
 * @param[out] destination_scan	frequencies and angles of sources detected will be stored here
 *
 * @return number our sources which were found, 0 if no sources are present
 */
//uint8_t getSourcesAndCheckKillerWhales(Destination *destination_scan, Destination *destination)
//{
//	uint16_t nb_sources = 0;
//	bool keepScanning = true;
//	bool wasThereAKillerWhale = false;
//
//	//We scan until there are no killer whales
//	while(keepScanning==true){
//		keepScanning = false; //We will set keepScanning to true if we see a killer whale
//
//		//We get the number of sources and have audio_processing store all found sources in destination_scan
//		nb_sources = audioP_findSources(destination_scan, destination);
//
//		if(nb_sources==0){
//			//if we detected a killer whale at least once, it means we set the robot moving so we have to stop here
//			if(wasThereAKillerWhale==true){
//				travCtrl_stopMoving();
//			}
//			return nb_sources;
//		}
//
//		//we scan each source and check if it's frequency is within killer whale 🐋 range
//		for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++) {
//
//			if(AUDIOP__KILLER_WHALE_FREQ_LOW < destination_scan[source_counter].freq && destination_scan[source_counter].freq < AUDIOP__KILLER_WHALE_FREQ_HIGH ){
//
//				keepScanning=true;
//				comms_printf( "Oh nooooooo... There is a killer whale 🐋 and I'm leaaaaaviiiiing !!!!\n\r");
//				wasThereAKillerWhale = true;
//
//				//We invert the angle (+/- 180°) of the killer whale direction and set the robot to it
//				if(destination_scan[source_counter].angle >=0){
//					travelCtrl_goToAngle(destination_scan[source_counter].angle-180);
//				}
//				else{
//					travelCtrl_goToAngle(destination_scan[source_counter].angle+180);
//				}
//			}
//		}
//	}
//
//	//if we detected a killer whale at least once, it means we set the robot moving so we have to stop here
//	if(wasThereAKillerWhale==true){
//		travCtrl_stopMoving();
//	}
//	return nb_sources;
//}

/*
 * @brief prints the sources and their properties to the bluetoot serial
 *
 * @param[in] nb_sources	how many sources are in destination_scan array
 * @param[in] destination_scan			array with properties of detected sources
 */



/*===========================================================================*/
/* Code to protect against smashing */
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
