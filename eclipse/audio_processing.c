/*
 * audio_processing.c
 *
 *  Created on: Apr 5, 2020
 *      Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: Provides all functions for listening to sounds, finding sources, frequencies and their direction
 * Functions prefix for public functions in this file: audioP_
 */
#include <ch.h>
#include <hal.h>
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <comms.h>
#include <fft.h>
#include <arm_math.h>

/*
 * Definitions
 */
//Program parameters
#define FFT_FREQ_MAX						1011						//Corresponding to 200Hz, upper limit of scanned freq.
#define FFT_FREQ_MIN						945						//Corresponding to 1200Hz, lower limit of scanned freq
#define PHASE_DIF_LIMIT					75.569					//Max arg dif for all freq. below 1200Hz, in deg
#define KILLER_FREQ						959						//Corresponding to 1000Hz, freq for killer whale
#define	FREQ_THD							3						//Threshold corresponding to 45Hz
#define AMPLI_THD						15000					//Threshold for peak-ampli
#define NB_ERROR_DETECTED_MAX			15						//Nb. of error scans before we assume that a source is not anymore available
#define EMA_WEIGHT						0.2						//range [0,1], if smaller past angles have more weight

//Microphone constants
#define FFT_SIZE 						1024
#define CMPX_VAL							2
#define CMPX_PART						1
#define LEFT_MIC							1
#define BACK_MIC							2
#define FRONT_MIC						3
#define NB_OF_MIC						4
#define NB_MIC_PAIR						2						//Two pairs of mic: left-right, back-front

//Physical constants
#define SPEED_SOUND						343						//[m/s]
#define EPUCK_MIC_DISTANCE				0.06						//Distance between two mic in [m]

//Number constants
#define ZERO								0
#define ONE								1
#define TWO_PI							6.2832
#define DEG90							90
#define DEG270							270
#define DEG360							360

//Scanning and writing mode constants
#define PEAK_MODE_SMALLER				0
#define PEAK_MODE_EXCHANGE				1
#define PEAK_MODE_REPLACE				2
#define PEAK_MODE_DO_NOTHING				3
#define WRITING_MODE_HIGHER				0
#define WRITING_MODE_LOWER				1
#define WRITING_MODE_EQUAL				2
#define WRITING_MODE_SOURCE				3
#define WRITING_MODE_ZERO				4


/*
 * Structure for sources
 * @note: Freq is not in Hz!
 */
typedef struct Sources {
	uint16_t freq;
	float ampli;
} Source;

/*
 * Semaphore
 */
static BSEMAPHORE_DECL(audioBufferIsReady, ONE);


/*
 * Static Variables
 */

//Audio buffer: 2*FFT_SIZE because arrays contain complex numbers (real + imaginary)
static float mic_buffer_left[CMPX_VAL*FFT_SIZE];
static float mic_buffer_right[CMPX_VAL*FFT_SIZE];
static float mic_buffer_back[CMPX_VAL*FFT_SIZE];
static float mic_buffer_front[CMPX_VAL*FFT_SIZE];

//arrays used to save the state of the mic audio buffer (double buffering)
//to avoid modifications of the buffer while analyzing it
static float mic_data_right[CMPX_VAL * FFT_SIZE];
static float mic_data_left[CMPX_VAL * FFT_SIZE];
static float mic_data_front[CMPX_VAL * FFT_SIZE];
static float mic_data_back[CMPX_VAL * FFT_SIZE];


//Static variables to memories freq, ampli and number of sources
static Source source[AUDIOP__NB_SOURCES_MAX];
static uint8_t nb_sources;

/*===========================================================================*/
/* Internal functions definitions             */
/*===========================================================================*/

/*
* @brief Callback for when the demodulation of the four microphones is done.
* @note : Sampling freq of mic: 16kHz. Every 10ms we get 160 samples per mic
*			We fill the samples buffers to reach 1024 samples to calculate the FFTs
*
* @parameter [in] data  			Buffer containing 4 times 160 samples. the samples are sorted by mic:
*							 		[micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
* @parameter [in] num_samples	Tells how many data we get in total (should always be 640)
*/
void audio_processAudioData(int16_t *data, uint16_t num_samples);

/*
 * @brief Copies the mic buffer and calculates FFT, determines sound peaks and writes them into source-array
 * @note : Waits until a buffer is full with 1024 samples
 */
void audio_analyseSpectre(void);

/*
 * @brief Calculates FFT and its amplitude of the for mic
 * 			FFT is saved in mic_data and amplitude in mic_ampli
 * @param[out] mic_data_xxx		4 audio data clip from the four mics, real sound values will be replaced by the complex fft values
 * @param[out] mic_ampli_left	1 empty arrays, to store the amplitudes of the fft
 */
void audio_CalculateFFT(float *mic_ampli_left);

/*
 * @brief 	finds loudest audio sources in sound clip, and their frequencies and amplitudes
 * @note 	this function updates file scope vars nb_sources and source (array) for later functions,
 * @note		Sources are first memorized in source_initial-array and then put in source-array if no error occured
 * @note		lowest_freq[FFTspace] is in source[0] ,highest_freq[FFTspace] is in source[nb]
 *
 * @param[in] mic_ampli			array of amplitudes for all frequencies
 *
 * @return	error codes AUDIOP__SUCCESS if all good, AUDIOP__ERROR otherwise
 */
uint16_t audio_Peak(float *mic_ampli);

/*
 * @brief 	Will search for peak amplitudes and puts them into source_initial-array
 * @note		source_initial-array is still sorted by its amplitude:
 * 			lowest_ampli is in source_init[0] ,highest_ampli[FFTspace] is in source[nb]
 *
 * @param[out] source_init 			pointer to a source_initial-array, where the found sources will be stored
 * @param[out] nb_sources_init 		pointer to where the number corresponding to how many sources were found should be stored.
 * @param[in] mic_ampli				array of amplitudes for all frequencies
 *
 * @return 					error codes AUDIOP__SUCCESS if all ok, AUDIOP__ERROR if error somewhere...
 */
int16_t audio_PeakScan(Source *source_init, uint8_t *nb_sources_init, float *mic_ampli);

/*
 * @brief	Changes source_initial-array corresponding to peak_mode and source_exchange
 *
 *  	@param[in] source_exchange 	indicates position of replacement (useful only if exchange or replace needed)
 *  	@param[in] freq_counter		indicates frequency of new source
 *  	@paraim[in] mic_ampli		amplitude at frequency: mic_ampli[freq_counter]
 *  @param[in] peak_mode			indicates which type of manipulation on the source_initial-array needs to be done. Possible cases are :
 *  								peak_mode=PEAK_MODE_EXCHANGE:	new ampli is bigger than smallest one of source array,
 *  																source_exchange indicates new position
 *  								peak_mode=PEAK_MODE_SMALLER: 	new ampli is smaller than all of source array,
 *  																should shift all array members to insert new one at index 0
 *  								peak_mode=PEAK_MODE_REPLACE: 	freq_difference<FREQ_THD and one value of source array has to be replaced
 *  																source_exchange indicates position of replacement
 *  	@param[out] source_init 		pointer to source_initial-array
 *  	@param[ou] nb_sources_init	point to number of sources stored in source_init array, if the new source is in fact added it will be incremented !
 *
 *  	@return						error codes AUDIOP__SUCCESS if all ok, AUDIOP__ERROR if error somewhere...
 */
int16_t audio_PeakChange(int8_t source_exchange, uint16_t freq_counter, float mic_ampli, uint8_t peak_mode, Source *source_init, uint8_t *nb_sources_init);

/*
 * @brief	Writes the changes evaluated in audio_PeakScan into source_initial-array
 * @note		Only fct that writes into source_init-array !
 *
 *  @param[in] source_counter 	indicates position of replacement
 *  @param[in] writing_mode		executes changes depending on peak_mode from audio_PeakScan
 *  @param[in] freq_counter		indicates frequency of new source
 *  @paraim[in] mic_ampli		amplitude at frequency: mic_ampli[freq_counter]
 *  @param[out] source_init 		pointer to source_initial-array
 *
 * @return		AUDIOP__SUCCESS if no errors, AUDIOP__ERROR if error occurred
 */
int16_t audio_PeakWriteInit(uint8_t source_counter, uint8_t writing_mode, uint16_t freq_counter, float mic_ampli, Source *source_init);

/*
 * @brief	Writes the changes evaluated in audio_Peak into source-array
 * @note		Only fct that writes into source-array !
 *
 *  @param[in] source_counter 	indicates position of writing
 *  @param[in] writing_mode		indicates which type of manipulation on the source_initial-array needs to be done. Possible cases are :
 *  								writing_mode=WRITING_MODE_ZERO:		writes a zero for freq and ampli
 *  								writing_mode=WRITING_MODE_SOURCE:	writes corresponding value of source_init-array into source-array
 *  	@param[out] source_init 		pointer to source_initial-array
 *
 * @return		AUDIOP__SUCCESS if no errors, AUDIOP__ERROR if error occurred
 */
int16_t audio_PeakWriteSource(uint8_t source_counter, uint8_t writing_mode,  Source *source_init);

/*
 * @brief	Sorts soure_initial-array by its frequency
 * @note		source_init[ZERO]=smallest_freq, source_init[NB_SOURCES]=highest_freq in FFT-space
 *
 * 	@param[out] source_init 		pointer to source_initial-array
 * 	@param[out] nb_source_init 	number of sources of source_initial-array
 * 	@param[in] mic_ampli			array of amplitudes for all frequencies
 *
 * @return		AUDIOP__SUCCESS if no errors, AUDIOP__ERROR if error occurred in audio_peakWriteInit
 */
int16_t audio_PeakBubblesort(Source *source_init, uint8_t nb_sources_init, float *mic_ampli);

/*
 * @brief calculates the angle of a given source
 *
 *  @param[in] source_index	index of source of which the angle is calculated
 *
 * @return	direction angle of source_index, between -180° and 180°, or AUDIOP__ERROR if there was an error
 */
int16_t audio_determineAngle(uint8_t source_index);

/*
 * @brief	Calculates the phase shift between mic one and mic two
 *
 *  @param[in] mic_data1		pointer to FFT values of mic one
 *  @param[in] mic_data2		pointer to FFT values of mic two
 *  @param[in] source_index	index of source of which the angle is calculated
 *
 * @return	AUDIOP__ERROR if error was detected, phase difference in degree if no error was detected
 */
int16_t audio_DeterminePhase(float *mic_data1, float *mic_data2, uint8_t source_index);

/*
 * @brief	Convert angle from radians into degrees
 *
 *  @param[in] rad		angle in radians
 *
 * @return	converted angle in degrees
 */
uint16_t audio_ConvertRad(float rad);

/*
 * @brief	Converts phase shift into angle
 * @note		freq is in Hz!
 *
 *  @param[in] arg		phase shift that has to be converted
 *  @param[in] freq		frequency of the source in Hz
 *
 * @return		angle in degrees
 */
uint16_t audio_ConvertPhase(int16_t arg, uint16_t freq);




/*===========================================================================*/
/* Public functions for setting/getting internal parameters           	  */
/*===========================================================================*/

void audioP_init()
{
	//starts the microphones processing thread.
	//it calls the callback given in parameter when samples are ready
	mic_start(&audio_processAudioData);
}

uint16_t audioP_analyseSources(Destination *destination_scan)
{
	bool errorDetected = true;

	while(errorDetected){
		errorDetected = false;

		audio_analyseSpectre();

		for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++) {

			if(abs(KILLER_FREQ-source[source_counter].freq) < FREQ_THD){
				if(audio_determineAngle(source_counter) != AUDIOP__ERROR){
					return AUDIOP__KILLER_WHALE_DETECTED;
				}
				else{
					errorDetected = true;
					break;
				}
			}

			destination_scan[source_counter].angle = audio_determineAngle(source_counter);
			if(destination_scan[source_counter].angle != AUDIOP__ERROR){
				destination_scan[source_counter].freq = source[source_counter].freq;
			}
			else{
				errorDetected = true;
			}
		}
	}

	return nb_sources;
}

uint16_t audioP_analyseDestination(Destination *destination)
{
	for(uint8_t error_counter = ZERO; error_counter<NB_ERROR_DETECTED_MAX; error_counter++){

		audio_analyseSpectre();

		for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++){

			if(abs(KILLER_FREQ-source[source_counter].freq) < FREQ_THD){
				if(audio_determineAngle(source_counter) != AUDIOP__ERROR){
					return AUDIOP__KILLER_WHALE_DETECTED;
				}
			}

			if(abs(destination->freq-source[source_counter].freq) < FREQ_THD){
				destination->angle = audio_determineAngle(source_counter);
				if(destination->angle != AUDIOP__ERROR){
					destination->freq = source[source_counter].freq;
					return AUDIOP__SUCCESS;
				}
			}

		}
	}

	return AUDIOP__SOURCE_NOT_FOUND;
}

uint16_t audioP_analyseKiller(Destination *killer)
{
	for(uint8_t error_counter = ZERO; error_counter<NB_ERROR_DETECTED_MAX; error_counter++){

		audio_analyseSpectre();

		for (uint8_t source_counter = 0; source_counter < nb_sources; source_counter++){

			if(abs(KILLER_FREQ-source[source_counter].freq) < FREQ_THD){
				killer->angle = audio_determineAngle(source_counter);
				if(killer->angle != AUDIOP__ERROR){
					killer->freq = source[source_counter].freq;
					return AUDIOP__SUCCESS;
				}
				else{
					break;
				}
			}
		}
	}

	return AUDIOP__SOURCE_NOT_FOUND;
}

/*
 * Convert the FFT value into a real frequency [Hz]
 */
uint16_t audioP_convertFreq(uint16_t freq)
{
	freq = (int) (15611 - 15.244*freq);
	return freq;
}




/*===========================================================================*/
/* Private functions              											*/
/*===========================================================================*/

void audio_processAudioData(int16_t *data, uint16_t num_samples)
{
	static uint16_t samples_gathered 	= ZERO;
	uint16_t sample_counter				= ZERO;

	while(sample_counter<num_samples){
		if(samples_gathered<FFT_SIZE){
			mic_buffer_right[CMPX_VAL*samples_gathered] = data[sample_counter];
			mic_buffer_right[CMPX_VAL*samples_gathered+CMPX_PART] = ZERO;
			mic_buffer_left[CMPX_VAL*samples_gathered] = data[sample_counter+LEFT_MIC];
			mic_buffer_left[CMPX_VAL*samples_gathered+CMPX_PART] = ZERO;
			mic_buffer_back[CMPX_VAL*samples_gathered] = data[sample_counter+BACK_MIC];
			mic_buffer_back[CMPX_VAL*samples_gathered+CMPX_PART] = ZERO;
			mic_buffer_front[CMPX_VAL*samples_gathered] = data[sample_counter+FRONT_MIC];
			mic_buffer_front[CMPX_VAL*samples_gathered+CMPX_PART] = ZERO;
			sample_counter += NB_OF_MIC;
			samples_gathered++;
		}
		else{
			samples_gathered = ZERO;
			chBSemSignal(&audioBufferIsReady);
		}
	}
}


void audio_analyseSpectre(void)
{
	static float mic_ampli_left[FFT_SIZE];

	while(true){		//DOTOPING return error if its not working

		//Waits until enough sound samples are collected
		chBSemWait(&audioBufferIsReady);

		//Copy buffer to avoid conflicts
		arm_copy_f32(mic_buffer_left, mic_data_left, CMPX_VAL * FFT_SIZE);
		arm_copy_f32(mic_buffer_right, mic_data_right, CMPX_VAL * FFT_SIZE);
		arm_copy_f32(mic_buffer_back, mic_data_back, CMPX_VAL * FFT_SIZE);
		arm_copy_f32(mic_buffer_front, mic_data_front, CMPX_VAL * FFT_SIZE);

		//Calculate FFT of sound signal, stores back inside mic_data_xxx for frequencies, and mic_ampli_xxx for amplitudes
		audio_CalculateFFT(mic_ampli_left);

		if(audio_Peak(mic_ampli_left) != AUDIOP__ERROR){	//Peak calculation was successful: source array was calculated with success
			break;
		}
	}
}

void audio_CalculateFFT(float *mic_ampli_left)
{
	doFFT_optimized(FFT_SIZE, mic_data_left);
	doFFT_optimized(FFT_SIZE, mic_data_right);
	doFFT_optimized(FFT_SIZE, mic_data_back);
	doFFT_optimized(FFT_SIZE, mic_data_front);
	arm_cmplx_mag_f32(mic_data_left, mic_ampli_left, FFT_SIZE);
}

uint16_t audio_Peak(float *mic_ampli)
{
	uint8_t source_counter						= ZERO;
	uint8_t nb_sources_init						= ZERO;
   	Source source_init[AUDIOP__NB_SOURCES_MAX]	= {0};

   	//Find sources sorted by their peak amplitudes and set their frequency into source_init array. Use by convention mic_ampli_left but could be any of the four mics.
   	if(audio_PeakScan(source_init, &nb_sources_init, mic_ampli)==AUDIOP__ERROR){
		return AUDIOP__ERROR;
	}

	if (nb_sources_init > AUDIOP__NB_SOURCES_MAX) {
#ifdef DEBUG_AUDIO
	comms_printf(UART_PORT_STREAM, "ERROR audio_Peak:		nb_sources is bigger than NB_SOURCES_MAX \n\r");
	comms_printf(UART_PORT_STREAM, "nb_sources_init = %u \n\r", nb_sources_init);
#endif
		return AUDIOP__ERROR;
	}

   	/*Bubblesort: sort source_init array accoording to frequencies : smallet frequency will be in source_init[0]->freq, ..., max frequency will be in source_init[nb_sources_init]*/
   	if(audio_PeakBubblesort(source_init, nb_sources_init, mic_ampli)==AUDIOP__ERROR){
   		return AUDIOP__ERROR;
   	}


   	nb_sources=nb_sources_init;
	for(source_counter=ZERO; source_counter<AUDIOP__NB_SOURCES_MAX; source_counter++){ 			//update file scoped source array with new sources in source_init and clear rest of array
		if(source_counter<nb_sources_init){
			audio_PeakWriteSource(source_counter, WRITING_MODE_SOURCE,  source_init);
		}
		else{ 																			//here we clear the rest of the file scope source  array
			audio_PeakWriteSource(source_counter, WRITING_MODE_ZERO,  source_init);
		}
	}

/*Error verification only for debugging! If code is working these errors will never happen.*/
#ifdef DEBUG_AUDIO
		for(source_counter=ZERO; source_counter<nb_sources_init; source_counter++){
			/*Error: Peak freq. out of range [150Hz,1200Hz], peak[i].freq is not in Hz!*/
			if((source_init[source_counter].freq>FFT_FREQ_MAX) || (source_init[source_counter].freq<FFT_FREQ_MIN)){
				chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : 	Max freq out of range ! \n\r");
				chprintf((BaseSequentialStream *)&SD3, "Source %d :			Peak freq = %d \n\r", source_counter, audioConvertFreq(source_init[source_counter].freq));
				return AUDIOP__ERROR;
			}
			/*Error: Peak ampli is too low*/
			if(source_init[source_counter].ampli<AMPLI_THD){
				chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : Max ampli too low ! \n\r");
				chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max ampli = %f \n\r", source_counter, source_init[source_counter].ampli);
				return AUDIOP__ERROR;
			}
			/*Error: Two peak freq are too close, difference<30Hz*/
			for(uint8_t i=ZERO; i<nb_sources_init; i++){
				if((i!=source_counter) && (abs(source_init[source_counter].freq-source_init[i].freq)<FREQ_THD)){

					chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : Max freq too close ! \n\r");
					chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max freq = %d \n\r", source_counter, audioConvertFreq(source_init[source_counter].freq));
					chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max freq = %d \n\r", i, audioConvertFreq(source_init[i].freq));
					return AUDIOP__ERROR;
				}
			}
		}
#endif

	return AUDIOP__SUCCESS;
}

int16_t audio_PeakScan(Source *source_init, uint8_t *nb_sources_init, float *mic_ampli)
{
	uint8_t source_counter						= ZERO;
	uint8_t source_exchange						= ZERO;
	uint8_t peak_mode							= ZERO;

		*nb_sources_init=ZERO;
		for(uint16_t freq_counter=FFT_FREQ_MIN; freq_counter<FFT_FREQ_MAX; freq_counter++){

			peak_mode=PEAK_MODE_DO_NOTHING;
			if(mic_ampli[freq_counter]>AMPLI_THD){
				source_exchange=ZERO;
				peak_mode=PEAK_MODE_SMALLER;
				for(source_counter=ZERO; source_counter<*nb_sources_init; source_counter++){
					if((freq_counter-(&source_init[source_counter])->freq)>FREQ_THD){
						if(mic_ampli[freq_counter]>(&source_init[source_counter])->ampli){
							source_exchange++;
							peak_mode=PEAK_MODE_EXCHANGE;
						}
					}
					else{
						if(mic_ampli[freq_counter]>(&source_init[source_counter])->ampli){
							source_exchange = source_counter;
							peak_mode=PEAK_MODE_REPLACE;
						}
						else{
							peak_mode=PEAK_MODE_DO_NOTHING;
						}
						break;
					}
				}
			}

			if(peak_mode != PEAK_MODE_DO_NOTHING){
				if(audio_PeakChange(source_exchange, freq_counter, mic_ampli[freq_counter], peak_mode, source_init, nb_sources_init)==AUDIOP__ERROR){
					return AUDIOP__ERROR;
				}
			}
		} //end for

	return AUDIOP__SUCCESS;
}

int16_t audio_PeakChange(int8_t source_exchange, uint16_t freq_counter, float mic_ampli, uint8_t peak_mode, Source *source_init, uint8_t *nb_sources_init)
{

	if(source_exchange<ZERO){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakExchange : source_exchange not valid! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "source_exchange = %d \n\r", source_exchange);
#endif
		return AUDIOP__ERROR;
	}
	else if(source_exchange>AUDIOP__NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "ERROR audio_PeakChange : source_counter out of range ! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "source_exchange = %d \n\r", source_exchange);
#endif
		return AUDIOP__ERROR;
	}

	switch(peak_mode){
		case PEAK_MODE_EXCHANGE:
			//if array is full, we will shift all sources under source_exchange down (deleting lowest one), and insert new at source_exchange
			//otherwise, if array is not full, we will go to case PEAK_MODE_SMALLER, to insert one and shift all up !
			if(*nb_sources_init==AUDIOP__NB_SOURCES_MAX){
				source_exchange--;
				for(uint8_t source_counter=ZERO; source_counter<source_exchange; source_counter++){
					if(audio_PeakWriteInit(source_counter, WRITING_MODE_LOWER, freq_counter, mic_ampli, source_init)==AUDIOP__ERROR){
						return AUDIOP__ERROR;
					}
				}
				if(audio_PeakWriteInit(source_exchange, WRITING_MODE_EQUAL, freq_counter, mic_ampli, source_init)==AUDIOP__ERROR){
					return AUDIOP__ERROR;
				}
			}
		case PEAK_MODE_SMALLER:
			//we need to insert new source at bottom of array, shifting all up, if it is not already full.
			if(*nb_sources_init<AUDIOP__NB_SOURCES_MAX){ //shift all up and insert lowest one at bottom (only if not already full)
				for(uint8_t source_counter=*nb_sources_init; source_counter>source_exchange; source_counter--){
					if(audio_PeakWriteInit(source_counter, WRITING_MODE_HIGHER, freq_counter, mic_ampli, source_init)==AUDIOP__ERROR){
						return AUDIOP__ERROR;
					}
				}
				if(audio_PeakWriteInit(source_exchange, WRITING_MODE_EQUAL, freq_counter, mic_ampli, source_init)==AUDIOP__ERROR){
					return AUDIOP__ERROR;
				}
				*nb_sources_init = *nb_sources_init + ONE;
			}
			break;

		case PEAK_MODE_REPLACE:
			//we shift all sources up from source exchange, but not add new source...
			while((source_exchange<(*nb_sources_init-ONE)) && (mic_ampli>(&source_init[source_exchange+ONE])->ampli)){
				if(audio_PeakWriteInit(source_exchange, WRITING_MODE_LOWER, freq_counter, mic_ampli, source_init)==AUDIOP__ERROR){
					return AUDIOP__ERROR;
				}

				source_exchange++;
			}
			if(audio_PeakWriteInit(source_exchange, WRITING_MODE_EQUAL, freq_counter, mic_ampli, source_init)==AUDIOP__ERROR){
				return AUDIOP__ERROR;
			}
			break;

		default:
#ifdef DEBUG_AUDIO
			chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakExchange : peak_mode invalid ! \n\r");
#endif
			return AUDIOP__ERROR;
	}
	return AUDIOP__SUCCESS;
}

int16_t audio_PeakWriteInit(uint8_t source_counter, uint8_t writing_mode, uint16_t freq_counter, float mic_ampli, Source *source_init)
{
	switch(writing_mode){
		case WRITING_MODE_HIGHER:
			if(source_counter>AUDIOP__NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
				chprintf((BaseSequentialStream *)&SD3, "ERROR audio_PeakWriteInit : source_counter out of range ! \n\r");
				chprintf((BaseSequentialStream *)&SD3, "sourc_counter = %d \n\r", source_counter);
#endif
				return AUDIOP__ERROR;
			}

			(&source_init[source_counter])->freq = (&source_init[source_counter-ONE])->freq;
			(&source_init[source_counter])->ampli = (&source_init[source_counter-ONE])->ampli;

			break;

		case WRITING_MODE_LOWER:
			if(source_counter>=AUDIOP__NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
				chprintf((BaseSequentialStream *)&SD3, "ERROR audio_PeakWriteInit : source_counter out of range ! \n\r");
				chprintf((BaseSequentialStream *)&SD3, "sourc_counter = %d \n\r", source_counter);
#endif
				return AUDIOP__ERROR;
			}

			(&source_init[source_counter])->freq = (&source_init[source_counter+ONE])->freq;
			(&source_init[source_counter])->ampli = (&source_init[source_counter+ONE])->ampli;

			break;

		case WRITING_MODE_EQUAL:
			if(source_counter>AUDIOP__NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
				chprintf((BaseSequentialStream *)&SD3, "ERROR audio_PeakWriteInit : source_counter out of range ! \n\r");
				chprintf((BaseSequentialStream *)&SD3, "sourc_counter = %d \n\r", source_counter);
#endif
				return AUDIOP__ERROR;
			}

			(&source_init[source_counter])->freq = freq_counter;
			(&source_init[source_counter])->ampli = mic_ampli;
			break;
		default:
#ifdef DEBUG_AUDIO
					chprintf((BaseSequentialStream *)&SD3, "ERROR audio_PeakWriteInit : writing_mode invalid ! \n\r");
#endif
					return AUDIOP__ERROR;

	}
	return AUDIOP__SUCCESS;
}

int16_t audio_PeakWriteSource(uint8_t source_counter, uint8_t writing_mode,  Source *source_init)
{
	if(source_counter>AUDIOP__NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "ERROR audio_PeakWriteSource : source_counter out of range ! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "sourc_counter = %d \n\r", source_counter);
#endif
		return AUDIOP__ERROR;
	}

	switch(writing_mode){
		case WRITING_MODE_SOURCE:
			source[source_counter].freq = (&source_init[source_counter])->freq;
			source[source_counter].ampli = (&source_init[source_counter])->ampli;
			break;
		case WRITING_MODE_ZERO:
			source[source_counter].freq = ZERO;
			source[source_counter].ampli = ZERO;
			break;
		default:
#ifdef DEBUG_AUDIO
			chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakExchange : peak_mode invalid ! \n\r");
#endif
			return AUDIOP__ERROR;
	}

	return AUDIOP__SUCCESS;
}

int16_t audio_PeakBubblesort(Source *source_init, uint8_t nb_sources_init, float *mic_ampli)
{
	uint16_t tmp_freq							= ZERO;
	float tmp_ampli								= ZERO;

	for (uint8_t source_counter=ONE; source_counter<nb_sources_init ; source_counter++){
		for (uint8_t i=ZERO; i<nb_sources_init-source_counter ; i++){
			if ((&source_init[i])->freq > (&source_init[i+ONE])->freq){
				tmp_freq = (&source_init[i])->freq;
				tmp_ampli = (&source_init[i])->ampli;
				if(audio_PeakWriteInit(i, WRITING_MODE_LOWER, i, mic_ampli[i], source_init)==AUDIOP__ERROR){
					return AUDIOP__ERROR;
				}
				(&source_init[i+ONE])->freq = tmp_freq;
				(&source_init[i+ONE])->ampli = tmp_ampli;
			}
		}
	}
	return AUDIOP__SUCCESS;
}

int16_t audio_determineAngle(uint8_t source_index)
{
	int16_t arg_dif_left_right							= ZERO;
	int16_t arg_dif_back_front							= ZERO;
	int16_t arg_dif										= ZERO;
	static uint8_t ema_counter[AUDIOP__NB_SOURCES_MAX];
	static int16_t ema_arg_dif[AUDIOP__NB_SOURCES_MAX];

	/*Calculate the angle shift with respect to the central axe of the robot*/
	arg_dif_left_right = audio_DeterminePhase(mic_data_left, mic_data_right, source_index);
	arg_dif_back_front = audio_DeterminePhase(mic_data_back, mic_data_front, source_index) ;

	/*Verify if there was an error in audio_DeterminePhase*/
	if(arg_dif_left_right==AUDIOP__ERROR || arg_dif_back_front==AUDIOP__ERROR){
		return AUDIOP__ERROR;
	}

	/*Convert phase shift into angle, provide freq in Hz to audioConvertFreq*/
	arg_dif_left_right = audio_ConvertPhase(arg_dif_left_right, audioP_convertFreq(source[source_index].freq));
	arg_dif_back_front = audio_ConvertPhase(arg_dif_back_front, audioP_convertFreq(source[source_index].freq));

	/*Determine plane of operation and averaging pairs of mic*/
	if((arg_dif_left_right>=ZERO) && (arg_dif_back_front>=ZERO)){
		arg_dif = (int16_t) ((arg_dif_left_right-arg_dif_back_front+DEG90)/NB_MIC_PAIR);
	}
	else if((arg_dif_left_right>ZERO) && (arg_dif_back_front<ZERO)){
		arg_dif = (int16_t) ((-arg_dif_left_right-arg_dif_back_front+DEG270)/NB_MIC_PAIR);
	}
	else if((arg_dif_left_right<ZERO) && (arg_dif_back_front>ZERO)){
		arg_dif = (int16_t) ((arg_dif_left_right+arg_dif_back_front-DEG90)/NB_MIC_PAIR);
	}
	else{
		arg_dif = (int16_t) ((-arg_dif_left_right+arg_dif_back_front-DEG270)/NB_MIC_PAIR);
	}

	/*Exponential Moving Average (EMA); EMA_WEIGHT range: [0,1], if smaller past results have more weight*/
	if(ema_counter[source_index]==ZERO){
		ema_arg_dif[source_index] = arg_dif;
		ema_counter[source_index]++;
	}
	else if(((ema_arg_dif[source_index]<-DEG90) && (arg_dif>DEG90)) || ((ema_arg_dif[source_index]>DEG90) && (arg_dif<-DEG90))){				//Jump from 180° to -180° or inverse
		ema_arg_dif[source_index] = arg_dif;
	}
	else{
		ema_arg_dif[source_index] = (int16_t) (arg_dif*EMA_WEIGHT + ema_arg_dif[source_index]*(ONE-EMA_WEIGHT));
	}

	return ema_arg_dif[source_index];
}

int16_t audio_DeterminePhase(float *mic_data1, float *mic_data2, uint8_t source_index)
{
	float phase1						=ZERO;								//in rad [-pi,+pi]
	float phase2						=ZERO;								//in rad [-pi,+pi]
	int16_t phase_dif				=ZERO;								//in degrees [-180°,+180°]
#ifdef DEBUG_AUDIO
	int16_t phase1_deg				=ZERO;								//in degrees [-180°,+180°]
	int16_t phase2_deg				=ZERO;								//in degrees [-180°,+180°]
#endif

	/*Calculate phase shift between the signal of mic1 and mic2; atan2f(float y, float x) returns float arctan(y/x) in rad [-pi,+pi]*/
	phase1 = atan2f(mic_data1[ ((CMPX_VAL*source[source_index].freq)+ONE) ], mic_data1[ (CMPX_VAL*source[source_index].freq) ]);
	phase2 = atan2f(mic_data2[ ((CMPX_VAL*source[source_index].freq)+ONE) ], mic_data2[ (CMPX_VAL*source[source_index].freq) ]);
	phase_dif = audio_ConvertRad(phase1-phase2);

	/*Error: Phase out of range; Outside of [-pi,+pi]*/
	if(phase1>PI || phase1<(-PI) || phase2>PI || phase2<(-PI)){
#ifdef DEBUG_AUDIO
		phase1_deg = audio_ConvertRad(phase1);
		phase2_deg = audio_ConvertRad(phase2);
		chprintf((BaseSequentialStream *)&SD3, "Error: 		phase1 or Ar2 out of range! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Source %d :		Phase1 (rad/deg): 		%f 		%d\n\r", source_index, phase1, phase1_deg);
		chprintf((BaseSequentialStream *)&SD3, "Source %d :		Phase2 (rad/deg): 		%f 		%d\n\r", source_index, phase2, phase2_deg);
#endif
		return AUDIOP__ERROR;
	}

	/*Error: Arg dif out of range; Max arg dif for all freq. below 1200Hz*/
	if(phase_dif>PHASE_DIF_LIMIT || phase_dif<(-PHASE_DIF_LIMIT)){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "Error: 		Phase dif out of range! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Source %d:		Phase dif (deg):		%d\n\r", source_index, phase_dif);
#endif
		return AUDIOP__ERROR;
	}

	return phase_dif;
}

uint16_t audio_ConvertRad(float rad)
{
	uint16_t degree = ZERO;
	degree = (int16_t) (360*(rad)/TWO_PI);
	return degree;
}

uint16_t audio_ConvertPhase(int16_t arg, uint16_t freq)
{
	arg = (int16_t) ((SPEED_SOUND*DEG90*arg)/(freq*EPUCK_MIC_DISTANCE*DEG360));

	/*Set max angle if arg overshoots 90°*/
	if(arg>DEG90){
		arg = DEG90;
	}
	if(arg<-DEG90){
		arg = -DEG90;
	}

	return arg;
}
