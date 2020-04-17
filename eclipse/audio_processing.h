#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

/*Enable for Debugging audio_processing*/
#define DEBUG_AUDIO

#define FFT_SIZE 						1024
#define ERROR_AUDIO						9999						//Error number
#define SUCCESS_AUDIO					1
#define NB_SOURCES						1						//Number of sources

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
*	Callback called when the demodulation of the four microphones is done.
*
*	Sampling freq of mic: 16kHz
*	Every 10ms we get 160 samples per mic
*	Fill the samples buffers to reach 1024 samples to calculate the FFTs
*
*	Parameters :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by mic:
*							 [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples		Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples);

/*
 * Determines the peak freq of all sources
 */
void audioDetermineSource(float *mic_data, float *mic_ampli, uint16_t *source);

/*
 * Determines the direction of the sound
 */
int16_t audioAnalyseDirection(float *mic_data_left, float *mic_data_right, float *mic_data_back, float *mic_data_font,
							float *mic_ampli_left, float *mic_ampli_right, float *mic_ampli_back, float *mic_ampli_front, uint8_t source_index);

/*
 * Determines the phase shift
 */
int16_t audioCalculatePhase(float *mic_data1, float *mic_data2, float *mic_ampli1, float *mic_ampli2, uint16_t *freq_index, uint8_t source_index);

/*
 * Calculates NB_SOURCES peak values and sorts them after freq
 * Returns peak freq of source_index; source_index=ZERO: lowest_freq, source_index=NB_SOURCES-ONE: highest_freq
 */
uint16_t audioCalculatePeak(float *mic_ampli, uint8_t source_index);

/*
 * Swaps elements such that array is sorted by its amplitude
 * max_ampli[0] = smallest ampli, max_ampli[NB_SOURCES-1] = biggest ampli
 */
void audioPeakSwap(uint8_t swaps, uint16_t *max_freq, float *max_ampli);

/*
 * Convert angle from radian into degree
 */
uint16_t audioConvertRad(float rad);

/*
 * Converts the FFT value into a real frequency
 */
uint16_t audioConvertFreq(uint16_t freq);

/*
 * Converts phase shift into angle
 */
uint16_t audioConvertPhase(int16_t arg, uint16_t freq);

/*
*	Put the invoking thread into sleep until it can process the audio data
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
