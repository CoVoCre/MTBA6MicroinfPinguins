/*
 * travelController.h
 *
 *  Created on: April 2, 2020
 *      Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * and stops when an obstacle/the objective is reached (detection with proximity sensor).
 */
#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

/*Enable for Debugging audio_processing*/
//#define DEBUG_AUDIO

#define FFT_SIZE 						1024
#define ERROR_AUDIO						9999						//Error number //TODOPING there was a problem, one time we return a uint8_t so not possible
																//as it was 9999 and this bigger than uint8_t of audioGetNbSources function
																//so I set it to 99 as it shouldn't be that many sources !
#define SUCCESS_AUDIO					1
#define ERROR_AUDIO_SOURCE_NOT_FOUND		8888
#define NB_SOURCES_MAX					5						//Max 255 sources!
#define UNINITIALIZED_FREQ				0
#define UNINITIALIZED_INDEX				255

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

/*
 * Structure for each source
 * Freq is not in Hz!
 */
typedef struct Sources {
	uint16_t freq;
	float ampli;
} Source;

/*
 * Structure for destination source
 * Freq is not in Hz!
 */
typedef struct Destinations {
	uint8_t index;
	uint16_t freq;
	int16_t arg;
} Destination;


/*===========================================================================*/
/* Public functions definitions             */
/*===========================================================================*/

/*
* @brief Starts the microphones thread and audio aquisition
*/
void audioP_init(void);

/*
 * @brief 	acquires and analyses a sound clip (FFT_SIZE samples), to find peaks intensity sources and their corresponding frequencies
 *
 * @return	number of sources that were found emitting typical sound, or ERROR_AUDIO if there was an error somewhere //TODOPING (ask user to reset)
 */
uint16_t audio_analyseSpectre(void);

/*
 * @brief calculates the angle of a given source, given its index corresponding to one of previously found sources
 *
 * @param[in] source_index	index of source to find direction angle of
 *
 * @return	direction angle of source_index, between -179* and 180°, or ERROR_AUDIO if there was an error
 */
int16_t audio_determineAngle(uint8_t source_index);

/*
 * @brief get the last calculated angle for the destination source (identified by its frequency)
 * @param[out] pointer to destinatin structure, where index and freq of destinatin source are.
 * 					values might be changed, frequency will be the nearest found frequency closer than FREQ_THD,
 * 					and index if order of sources has changed
 * @return SUCCESS_AUDIO if all good, ERROR_AUDIO_SOURCE_NOT_FOUND if not available anymore, or ERROR_AUDIO if problem happened
 */
uint16_t audio_updateDirection(Destination *destination);

/*
 * Returns freq of source: source[source_index].freq
 * Rturns ERROR_AUDIO if source[source_index].freq=ERROR_AUDIO or source[source_index].freq=ZERO
 */
uint16_t audioGetSourceFreq(uint8_t source_index);

/*
 * Converts the FFT value into a real frequency
 */
uint16_t audioConvertFreq(uint16_t freq);

#endif /* AUDIO_PROCESSING_H */
