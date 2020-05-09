/*
 * travelController.h
 *
 *  Created on: April 1, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: Provides all functions for listening to sounds, finding sources, frequencies and their direction
 * Function prefix for public functions in this file: audioP_
 * Constant prefix for public constants in this file: AUDIOP__
 */
#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

/*===========================================================================*/
/* Constants definition for this library						               */
/*===========================================================================*/

//Program parameters
#define AUDIOP__NB_SOURCES_MAX				5						//Max 255 sources because nb_sources is uint8_t

//Returning state constants
#define AUDIOP__ERROR						9999						//Error number
#define AUDIOP__SUCCESS						1
#define AUDIOP__SOURCE_NOT_FOUND				8888
#define AUDIOP__KILLER_WHALE_DETECTED		6666

//Initialization constant
#define AUDIOP__UNINITIALIZED_FREQ			0


/*===========================================================================*/
/* Structures						 			                            */
/*===========================================================================*/

/*
 * Structure for destination source
 * Freq is not in Hz!
 */
typedef struct Destinations {
	uint16_t freq;
	int16_t angle;
} Destination;


/*===========================================================================*/
/* Public functions definitions            									 */
/*===========================================================================*/

/*
* @brief Starts the microphones thread and audio acquisition
*/
void audioP_init(void);

/*
 * @brief		scans the sound data and fills the destination_scan-array with all available sources
 * @note			verifies if there is a killer whale
 *
 *  @pram[out] destination_scan		array of structure Destination to pass over available sources to main
 *
 * @return	number of sources if no killer whale is detected and AUDIOP__KILLER_WHALE_DETECTED otherwise
 */
uint16_t audioP_analyseSources(Destination *destination_scan);

/*
 * brief		scans the sound data and updates the destination-structure of the target to which the robot is moving
 *
 *  @pram[out] destination		structure Destination to update the freq and the angle of the destination in main
 *
 * return	SUCCES_AUDIO if angle calculations were successful,
 * 			AUDIOP__SOURCE_NOT_FOUND if the source could not be found or angle calculations had errors for NB_ERROR_DETECTED_MAX times,
 * 			AUDIOP__KILLER_WHALE_DETECTED if a killer whale was detected
 */
uint16_t audioP_analyseDestination(Destination *destination);

/*
 * brief		scans the sound data for a killer whale and updates the killer-structure of the killer whale
 *
 *  @pram[out] killer		structure Destination to update the freq and the angle of the killer whale in main
 *
 * return	SUCCES_AUDIO if angle calculations were successful,
 * 			AUDIOP__SOURCE_NOT_FOUND if the source could not be found or angle calculations had errors for NB_ERROR_DETECTED_MAX times,
 */
uint16_t audioP_analyseKiller(Destination *killer);

/*
 * @brief	converts the frequency from the FFT-domain into a real frequency in Hz
 *
 *  @param[in] freq		frequency in the FFT-domain
 *
 * @return	real frequency in Hz
 */
uint16_t audioP_convertFreq(uint16_t freq);


#endif /* AUDIO_PROCESSING_H */
