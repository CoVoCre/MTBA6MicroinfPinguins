/*
 * travelController.h
 *
 *  Created on: April 2, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * and stops when an obstacle/the objective is reached (detection with proximity sensor).
 */
#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define FFT_SIZE 						1024
#define ERROR_AUDIO						9999						//Error number //TODOPING there was a problem, one time we return a uint8_t so not possible
																//as it was 9999 and this bigger than uint8_t of audioGetNbSources function
																//so I set it to 99 as it shouldn't be that many sources !
#define SUCCESS_AUDIO					1	//TODOPING are those defines all useful in other files ? For global defines maybe use AUDIOP__ in front ?
#define ERROR_AUDIO_SOURCE_NOT_FOUND		8888
#define AUDIOP__NB_SOURCES_MAX			5						//Max 255 sources!
#define AUDIOP__UNINITIALIZED_FREQ		0
#define AUDIOP__UNINITIALIZED_INDEX		255

#define	AUDIOP__FREQ_THD							3						//Threshold corresponding to 45Hz

#define AUDIOP__HZ_TO_FFT_FREQ(hz) 		( (10241000-656*hz)/10000 )

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
typedef struct Sources {	// TODOPING Should this be public ?
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
	int16_t angle;	//TODOPING change this to angle if it's just used as angle...
} Destination;


/*===========================================================================*/
/* Public functions definitions             */
/*===========================================================================*/

/*
* @brief Starts the microphones thread and audio aquisition
*/
void audioP_init(void);

/*
 * Converts the FFT value into a real frequency
 */
uint16_t audioP_convertFreq(uint16_t freq);

/*
 * @brief acquires and analyses a sound clip (FFT_SIZE samples), to find peaks intensity
 * 			sources and their corresponding frequencies and angles
 * @note this function retries until there are no errors so afterwards you
 * 			do not have to check for errors
 *
 * @param[out] destination_scan	array where found sources will be stored
 *
 * @return number of sources found
 */
uint8_t audioP_findSources(Destination *destination_scan);

#endif /* AUDIO_PROCESSING_H */
