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

#define KILLER_WHALE_FREQ_LOW AUDIOP__HZ_TO_FFT_FREQ(200)
#define KILLER_WHALE_FREQ_HIGH AUDIOP__HZ_TO_FFT_FREQ(300)

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
uint8_t getSourcesAndCheckKillerWhales(Destination *destination_scan);

void communicationUser(Destination *destination);

void moveTowardsTarget(Destination *destination);

void printSources(uint8_t numberOfSourcesDetected, Destination *destination_scan);

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
/* -----------------------  MAIN FUNCTION OF PROJECT -----------------------*/
/*===========================================================================*/

/*
 * @brief this is were the whole project begins
 * @note it actually never returns but the type int is mandatory
 * 			for convention purposes
 */
int main(void) {

	Destination destination;
	destination.index = AUDIOP__UNINITIALIZED_INDEX;
	destination.freq = AUDIOP__UNINITIALIZED_FREQ;
	destination.angle = 0;

	//Initialise chibios systems, hardware abstraction layer and memory protection
	halInit();
	chSysInit();
	mpu_init();

	//Start the serial communication over bluetooth
	comms_start();

	comms_printf("Welcome to the penguin simulator!\n\r\n\r");
	comms_printf("Our robot will try to beat penguins at their game,\n\r");
	comms_printf("that is identifying sounds of their chicks and going\n\r");
	comms_printf( "to one of them amongst many.\n\r");
	comms_printf( "You will decide which onoe the penguins goes to...\n\r\n\r");
	comms_printf( "But be aware of killer whales... !\n\r");

	comms_printf( "\n\r\n\rThe simulation will now begin.\n\r\n\r");

	// Initialise motor controller. It does not move yet, as it waits for an angle
	travCtrl_init(destReachedCB);

	// Initialise audio module, which starts listening to mics and acquiring audio data
	audioP_init();

	/* Infinite main thread loop. */
	while (1){

		communicationUser(&destination); //Scan sources and ask the user to select one... except if there is a killer whale !

		moveTowardsTarget(&destination); //Move towards the selected source, until it is reached or not found anymore

		chThdSleepMilliseconds(1000); //wait 1 second before restarting for final messages to finish sending to computer
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
	Destination destination_scan[AUDIOP__NB_SOURCES_MAX] = {0};	//we store all found sources properties here
	bool keepAsking = true;	//this is the while control variable, to ask and re ask until a source is selected

	/* Scans sources until there are no errors and no killer whales 🐋,
	 * and returns how many were found at that point */
	uint8_t numberOfSourcesDetected = getSourcesAndCheckKillerWhales(destination_scan);

	printSources(numberOfSourcesDetected, destination_scan);

	//we ask the user for a number or r (rescan), and keep asking until the the input is valid
	while(keepAsking == true){

		comms_printf( "Please enter the number of the source you want our little\n\r");
		comms_printf( "penguin to go to, or enter r to rescan sources.\n\r");
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
			else if(readNumber < numberOfSourcesDetected){
				keepAsking = false;	//Here we will actually have a good source
			}
			else{	//the number wasn't a valid source or there were 0 sources available and user didn't press r
				comms_printf( "It seems the number %u you just entered is not a valid source. Please try again !\n\r", readNumber);
			}
		}
		else{ //otherwise (user typed r) we rescan
			numberOfSourcesDetected = getSourcesAndCheckKillerWhales(destination_scan);
			printSources(numberOfSourcesDetected, destination_scan);
		}
	}

	destination->index = readNumber;
	destination->freq = destination_scan[destination->index].freq;
}

/*
 * @brief get all monofrequency sound sources and retries until there are no killer whales 🐋
 * @note if there is a killer whale, it will make the robot penguin run away from it
 *
 * @param[out] destination_scan	frequencies and angles of sources detected will be stored here
 *
 * @return number our sources which were found, 0 if no sources are present
 */
uint8_t getSourcesAndCheckKillerWhales(Destination *destination_scan)
{
	uint16_t nb_sources = 0;
	bool keepScanning = true;
	comms_printf( "Scanning for sources ...\n\r");

	//We scan until there are no killer whales
	while(keepScanning==true){
		keepScanning = false; //We will set keepScanning to true if we see a killer whale

		//We get the number of sources and have audio_processing store all found sources in destination_scan
		nb_sources = audio_findSources(destination_scan);

		if(nb_sources==0){
			break;
		}

		//we scan each source and check if it's frequency is within killer whale 🐋 range
		for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++) {

			if(KILLER_WHALE_FREQ_LOW < destination_scan[source_counter].freq && destination_scan[source_counter].freq < KILLER_WHALE_FREQ_HIGH ){

				keepScanning=true;
				comms_printf( "Oh nooooooo... There is a killer whale 🐋 and I'm leaaaaaviiiiing !!!!\n\r");

				//We invert the angle (+/- 180°) of the killer whale direction and set the robot to it
				if(destination_scan[source_counter].angle >=0){
					travelCtrl_goToAngle(destination_scan[source_counter].angle-180);
				}
				else{
					travelCtrl_goToAngle(destination_scan[source_counter].angle+180);
				}
			}
		}
	}

	return nb_sources;
}

/*
 * @brief prints the sources and their properties to the bluetoot serial
 *
 * @param[in] numberOfSourcesDetected	how many sources are in destination_scan array
 * @param[in] destination_scan			array with properties of detected sources
 */
void printSources(uint8_t numberOfSourcesDetected, Destination *destination_scan){
	/* Printing the available sources and their frequencies.
	 * When we get here we are sure there are no more errors and killer whales, so we do not check anymore */
	comms_printf( "The following sources are available: \n\r");
	if(numberOfSourcesDetected==0){
		comms_printf("    ...No sources we found...\n\r");

	}
	else{
		for (uint8_t source_counter = 0; source_counter < numberOfSourcesDetected; source_counter++) {
			comms_printf("Source %d :	 frequency =%u		angle =%d \n\r", source_counter,
					audio_ConvertFreq(destination_scan[source_counter].freq), destination_scan[source_counter].angle);
		}
	}
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
void moveTowardsTarget(Destination *destination)
{
	uint16_t nb_sources				= 0;
	Destination destination_scan[AUDIOP__NB_SOURCES_MAX]		= {0};	// here we will store all found sources properties
	bool isDestinationStillHere = false;

	/*Now we set robotMoving file variable to true, it will either be set to false here if
	 * we cannot find the destination source anymore, or by the destReachedCB function
	 * called from travelController when the robot encounters an obstacle. */
	robotMoving = true;

	while (robotMoving == true) {

		/*scanSources returns only when there are no killer whales
		 * (if killer whale is present, it will run away from it). */
		nb_sources = getSourcesAndCheckKillerWhales(destination_scan);

		//Now we check if the selected source is still available
		for(uint8_t source_counter = 0; source_counter<nb_sources; source_counter++){
			if( abs(destination->freq-destination_scan[source_counter].freq) < AUDIOP__FREQ_THD){
				isDestinationStillHere = true;
				destination->index=source_counter;
				destination->angle = destination_scan[destination->index].angle;
			}
		}

		/* When the destination selected isn't anymore available,
		 * we stop moving towards the source and thus exit the while loop.
		 * If 0 sources are found it also means the source is not available */
		if(isDestinationStillHere==false || nb_sources==0){
			comms_printf( "The source you selected is not available anymore, please select a new one.\n\r");
			robotMoving = false;
			travCtrl_stopMoving();
			break;	//exit robot moving while loop
		}

		if(robotMoving == true){	//we check robotMoving == true in case the source is not found or
								//destination was reached while we were withing robotMoving loop
			travelCtrl_goToAngle(destination->angle);
		}
	} //end of while robotMoving
}

/*===========================================================================*/
/* Code to protect against smashing
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
	chSysHalt("Stack smashing detected");
}
