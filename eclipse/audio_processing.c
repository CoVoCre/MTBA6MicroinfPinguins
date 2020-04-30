#include <ch.h>
#include <hal.h>
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
//#include <communications.h>
#include <fft.h>
#include <arm_math.h>

/*Definitions*/
#define CMPX_VAL							2
#define CMPX_PART						1
#define LEFT_MIC							1
#define BACK_MIC							2
#define FRONT_MIC						3
#define NB_OF_MIC						4
#define NB_MIC_PAIR						2						//Two pairs of mic: left-right, back-front

#define FREQ_MAX							945						//Corresponds to 1200Hz
#define FREQ_MIN							1015						//Corresponds to 150Hz
#define AMPLI_THD						15000					//Threshold for peak-ampli
#define	FREQ_THD							3						//Threshold corresponding to 45Hz

#define HALF_FFT_SIZE					512
#define PHASE_DIF_LIMIT					75.569					//Max arg dif for all freq. below 1200Hz, in deg
#define EMA_WEIGHT						0.2						//range [0,1], if smaller past results have more weight
#define SPEED_SOUND						343						//[m/s]
#define EPUCK_MIC_DISTANCE				0.06						//Distance between two mic in [m]

#define FALSE							0
#define TRUE								1
#define ZERO								0
#define ONE								1
#define TWO_PI							6.2832
#define DEG90							90
#define DEG270							270
#define DEG360							360

#define PEAK_MODE_SMALLER				0
#define PEAK_MODE_EXCHANGE				1
#define PEAK_MODE_REPLACE				2
#define PEAK_MODE_DO_NOTHING				3
#define WRITING_MODE_HIGHER				0
#define WRITING_MODE_LOWER				1
#define WRITING_MODE_EQUAL				2
#define WRITING_MODE_SOURCE				3
#define WRITING_MODE_ZERO				4
#define NB_STAB_CYCLES					8

#define EQUAL							1
#define UNEQUAL							0

/*Semaphore*/
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);


/*Static Variables*/

/*Audio buffer: 2*FFT_SIZE because arrays contain complex numbers (real + imaginary)*/
static float micLeft_cmplx_input[CMPX_VAL*FFT_SIZE];
static float micRight_cmplx_input[CMPX_VAL*FFT_SIZE];
static float micFront_cmplx_input[CMPX_VAL*FFT_SIZE];
static float micBack_cmplx_input[CMPX_VAL*FFT_SIZE];

/*Static structure to memories freq and ampli of each source*/
static Source source[NB_SOURCES_MAX];

static uint8_t nb_sources;

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
void processAudioData(int16_t *data, uint16_t num_samples)
{
	static uint16_t samples_gathered 	= ZERO;
	uint16_t sample_counter				= ZERO;

	while(sample_counter<num_samples){
		if(samples_gathered<FFT_SIZE){
			micRight_cmplx_input[CMPX_VAL*samples_gathered] = data[sample_counter];
			micRight_cmplx_input[CMPX_VAL*samples_gathered+CMPX_PART] = ZERO;
			micLeft_cmplx_input[CMPX_VAL*samples_gathered] = data[sample_counter+LEFT_MIC];
			micLeft_cmplx_input[CMPX_VAL*samples_gathered+CMPX_PART] = ZERO;
			micBack_cmplx_input[CMPX_VAL*samples_gathered] = data[sample_counter+BACK_MIC];
			micBack_cmplx_input[CMPX_VAL*samples_gathered+CMPX_PART] = ZERO;
			micFront_cmplx_input[CMPX_VAL*samples_gathered] = data[sample_counter+FRONT_MIC];
			micFront_cmplx_input[CMPX_VAL*samples_gathered+CMPX_PART] = ZERO;
			sample_counter += NB_OF_MIC;
			samples_gathered++;
		}
		else{
			samples_gathered = ZERO;
			chBSemSignal(&sendToComputer_sem);
		}
	}
}

/*
 * Calculates FFT and its amplitude of the for mic
 * FFT is saved in mic_data and amplitude in mic_ampli
 */
void audioCalculateFFT(float *mic_data_left, float *mic_data_right, float *mic_data_back, float *mic_data_front,
						float *mic_ampli_left, float *mic_ampli_right, float *mic_ampli_back, float *mic_ampli_front)
{
	doFFT_optimized(FFT_SIZE, mic_data_left);
	doFFT_optimized(FFT_SIZE, mic_data_right);
	doFFT_optimized(FFT_SIZE, mic_data_back);
	doFFT_optimized(FFT_SIZE, mic_data_front);
	arm_cmplx_mag_f32(mic_data_left, mic_ampli_left, FFT_SIZE);
	arm_cmplx_mag_f32(mic_data_right, mic_ampli_right, FFT_SIZE);
	arm_cmplx_mag_f32(mic_data_back, mic_ampli_back, FFT_SIZE);
	arm_cmplx_mag_f32(mic_data_front, mic_ampli_front, FFT_SIZE);
}

uint16_t audioGetSourceFreq(uint8_t source_index)
{
	if((source[source_index].freq==ERROR_AUDIO) || (source[source_index].freq==ZERO)){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "audioGetSourceFreq: source[source_index].freq==ERROR_AUDIO) || (source[source_index].freq==ZERO)	\n\r\n\r");
#endif
		return ERROR_AUDIO;
	}
	return source[source_index].freq;
}

uint8_t audioGetNbSources(void)
{
	if(nb_sources>NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
        		chprintf((BaseSequentialStream *)&SD3, "audioGetNbSources:	nb_sources>NB_SOURCES_MAX\n\r\n\r");
#endif
		return ERROR_AUDIO;
	}
	return nb_sources;
}


/*
 * Calculate angle of sound direction
 */
int16_t audioDetermineAngle(float *mic_data_left, float *mic_data_right, float *mic_data_back, float *mic_data_front, uint8_t source_index)
{
	int16_t arg_dif_left_right							= ZERO;
	int16_t arg_dif_back_front							= ZERO;
	int16_t arg_dif										= ZERO;
	static uint8_t ema_counter[NB_SOURCES_MAX];
	static int16_t ema_arg_dif[NB_SOURCES_MAX];

	/*Calculate the angle shift with respect to the central axe of the robot*/
	arg_dif_left_right = audioDeterminePhase(mic_data_left, mic_data_right, source_index);
	arg_dif_back_front = audioDeterminePhase(mic_data_back, mic_data_front, source_index) ;

	/*Verify if there was an error in audioDeterminePhase*/
	if(arg_dif_left_right==ERROR_AUDIO || arg_dif_back_front==ERROR_AUDIO){
		return ERROR_AUDIO;
	}

	/*Convert phase shift into angle, provide freq in Hz to audioConvertFreq*/
	arg_dif_left_right = audioConvertPhase(arg_dif_left_right, audioConvertFreq(source[source_index].freq));
	arg_dif_back_front = audioConvertPhase(arg_dif_back_front, audioConvertFreq(source[source_index].freq));

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

/*
 * Calculate phase difference between two mic.
 */
int16_t audioDeterminePhase(float *mic_data1, float *mic_data2, uint8_t source_index)
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
	phase_dif = audioConvertRad(phase1-phase2);

	/*Error: Phase out of range; Outside of [-pi,+pi]*/
	if(phase1>PI || phase1<(-PI) || phase2>PI || phase2<(-PI)){
#ifdef DEBUG_AUDIO
		phase1_deg = audioConvertRad(phase1);
		phase2_deg = audioConvertRad(phase2);
		chprintf((BaseSequentialStream *)&SD3, "Error: 		phase1 or Ar2 out of range! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Source %d :		Phase1 (rad/deg): 		%f 		%d\n\r", source_index, phase1, phase1_deg);
		chprintf((BaseSequentialStream *)&SD3, "Source %d :		Phase2 (rad/deg): 		%f 		%d\n\r", source_index, phase2, phase2_deg);
#endif
		return ERROR_AUDIO;
	}

	/*Error: Arg dif out of range; Max arg dif for all freq. below 1200Hz*/
	if(phase_dif>PHASE_DIF_LIMIT || phase_dif<(-PHASE_DIF_LIMIT)){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "Error: 		Phase dif out of range! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Source %d:		Phase dif (deg):		%d\n\r", source_index, phase_dif);
#endif
		return ERROR_AUDIO;
	}

	return phase_dif;
}

/*
 * Calculates NB_SOURCES peak values and sorts them after freq
 * Returns peak freq of source_index; source_index=ZERO: lowest_freq, source_index=NB_SOURCES-ONE: highest_freq
 */
uint16_t audioPeak(float *mic_ampli_left, Destination *destination)
{
	uint8_t source_counter						= ZERO;
	uint8_t nb_sources_init						= ZERO;
	static uint8_t nb_sources_change 			= ZERO;
	static uint8_t nb_sources_change_counter		= ZERO;
   	static Source source_init[NB_SOURCES_MAX];

   	/*Calculate source_init arry for each mic and retrun error if they are unequal*/
   	if(audioPeakScan(source_init, &nb_sources_init, mic_ampli_left)==ERROR_AUDIO){
		return ERROR_AUDIO;
	}

   	/*Bubblesort: max_freq[ZERO]=smallest_freq, max_freq[NB_SOURCES]=highest_freq*/
   	if(audioPeakBubblesort(source_init, nb_sources_init, mic_ampli_left)==ERROR_AUDIO){
   		return ERROR_AUDIO;
   	}
	chprintf((BaseSequentialStream *)&SD3, "Source0 :		freq = %d \n\r", source_init[0].freq);

//   	if(destination->index==UNINITIALIZED_INDEX){
//		nb_sources=nb_sources_init;
//   		for(source_counter=ZERO; source_counter<NB_SOURCES_MAX; source_counter++){
//   			audioPeakWriteSource(source_counter, WRITING_MODE_SOURCE,  source_init);
//   		}
//   	}
//   	else{
//   		/*If stable state is reached, write source_init into source array*/
//		if(nb_sources_init!=nb_sources_change){
//			nb_sources_change = nb_sources_init;
//			nb_sources_change_counter = NB_STAB_CYCLES;
//		}
//		else{
//			if(nb_sources_change_counter > ZERO){
//				nb_sources_change_counter--;
//			}
//			else{
//				nb_sources=nb_sources_init;
//				destination->index=UNINITIALIZED_INDEX;
//				for(source_counter=ZERO; source_counter<NB_SOURCES_MAX; source_counter++){
//					if(source_counter<nb_sources_init){
//						audioPeakWriteSource(source_counter, WRITING_MODE_SOURCE,  source_init);
//						if(abs(destination->freq-source[source_counter].freq)<FREQ_THD){
//							destination->freq=source[source_counter].freq;
//							destination->index=source_counter;
//						}
//					}
//					else{
//						audioPeakWriteSource(source_counter, WRITING_MODE_ZERO,  source_init);
//					}
//				}
//
//
//				if((nb_sources!=ZERO) && (destination->index==UNINITIALIZED_INDEX)){
//	#ifdef DEBUG_AUDIO
//					chprintf((BaseSequentialStream *)&SD3, "audioPeak: ERROR - destination freq is not anymore available! \n\r");
//	#endif
//					return ERROR_AUDIO_SOURCE_NOT_FOUND;
//				}
//			}
//		}
//	}
//
//	/*Error: Peak ampli is too low*/
//	if(source_init[destination->index].ampli<AMPLI_THD){
//#ifdef DEBUG_AUDIO
//		chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : Max ampli too low ! \n\r");
//		chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max ampli = %f \n\r", destination->index, source_init[destination->index].ampli);
//#endif
//		return ERROR_AUDIO;
//	}
//
//	/*Error: Peak freq. out of range [150Hz,1200Hz], peak[i].freq is not in Hz!*/
//	if((source_init[destination->index].freq>FREQ_MIN) || (source_init[destination->index].freq<FREQ_MAX)){						//Inverse logic because freq is not in Hz!
//#ifdef DEBUG_AUDIO
//		chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : 	Max freq out of range ! \n\r");
//		chprintf((BaseSequentialStream *)&SD3, "Source %d :			Peak freq = %d \n\r", destination->index, audioConvertFreq(source_init[destination->index].freq));
//#endif
//		return ERROR_AUDIO;
//	}
//
//   	/*Error: Two peak freq are two close, difference<30Hz*/
//	for(source_counter=ZERO; source_counter<nb_sources_init; source_counter++){
//   		for(uint8_t i=ZERO; i<nb_sources_init; i++){
//   			if((i!=source_counter) && (abs(source_init[source_counter].freq-source_init[i].freq)<FREQ_THD)){
//#ifdef DEBUG_AUDIO
//   				chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : Max freq too close ! \n\r");
//   				chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max freq = %d \n\r", source_counter, audioConvertFreq(source_init[source_counter].freq));
//   				chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max freq = %d \n\r", i, audioConvertFreq(source_init[i].freq));
//#endif
//   				return ERROR_AUDIO;
//   			}
//   		}
//	}

	return SUCCESS_AUDIO;
}

/*
 * Changing source array if necessary, array is sorted by ampli: source[0].=smallest_ampli, source[nb_sources].=biggest_ampli
 */
int16_t audioPeakScan(Source *source_init, uint8_t *nb_sources_init, float *mic_ampli)
{
	uint8_t source_counter						= ZERO;
	uint8_t source_exchange						= ZERO;
	uint8_t peak_mode							= ZERO;

		*nb_sources_init=ZERO;
		for(uint16_t freq_counter=HALF_FFT_SIZE; freq_counter<FFT_SIZE; freq_counter++){
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
				if(audioPeakChange(source_exchange, freq_counter, mic_ampli, peak_mode, source_init, nb_sources_init)==ERROR_AUDIO){
					return ERROR_AUDIO;
				}
			}
		}

return SUCCESS_AUDIO;
}


/*
 * Changes source array corresponding to peak_mode and source_exchange
 *  peak_mode=PEAK_MODE_EXCHANGE: new ampli is bigger than min one of source array
 *  		source_exchange: indicates new position
 *  peak_mode=PEAK_MODE_SMALLER: new ampli is smaller than all of source array
 *  peak_mode=PEAK_MODE_REPLACE: freq_difference<FREQ_THD and one value of source array has to be replaced
 *  		source_exchange: indicates position of replacement
 */
int16_t audioPeakChange(int8_t source_exchange, uint16_t freq_counter, float *mic_ampli, uint8_t peak_mode, Source *source_init, uint8_t *nb_sources_init)
{
	uint8_t source_counter = ZERO;

	if(source_exchange<ZERO){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakExchange : source_exchange not valid! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "source_exchange = %d \n\r", source_exchange);
#endif
		return ERROR_AUDIO;
	}

	switch(peak_mode){
		case PEAK_MODE_EXCHANGE:
			if(*nb_sources_init==NB_SOURCES_MAX){
				source_exchange--;
				for(source_counter=ZERO; source_counter<source_exchange; source_counter++){
					if(audioPeakWriteInit(source_counter, WRITING_MODE_LOWER, freq_counter, mic_ampli, source_init)==ERROR_AUDIO){
						return ERROR_AUDIO;
					}

				}
				if(audioPeakWriteInit(source_exchange, WRITING_MODE_EQUAL, freq_counter, mic_ampli, source_init)==ERROR_AUDIO){
					return ERROR_AUDIO;
				}
			}

		case PEAK_MODE_SMALLER:
			if(*nb_sources_init<NB_SOURCES_MAX){
				for(source_counter=*nb_sources_init; source_counter>source_exchange; source_counter--){
					if(audioPeakWriteInit(source_counter, WRITING_MODE_HIGHER, freq_counter, mic_ampli, source_init)==ERROR_AUDIO){
						return ERROR_AUDIO;
					}
				}
				if(audioPeakWriteInit(source_exchange, WRITING_MODE_EQUAL, freq_counter, mic_ampli, source_init)==ERROR_AUDIO){
					return ERROR_AUDIO;
				}
				*nb_sources_init = *nb_sources_init + ONE;
			}
			break;

		case PEAK_MODE_REPLACE:
			while((source_exchange<(*nb_sources_init-1)) && (mic_ampli[freq_counter]>(&source_init[source_counter+ONE])->ampli)){
				if(audioPeakWriteInit(source_exchange, WRITING_MODE_LOWER, freq_counter, mic_ampli, source_init)==ERROR_AUDIO){
					return ERROR_AUDIO;
				}
				source_exchange++;
			}
			if(audioPeakWriteInit(source_exchange, WRITING_MODE_EQUAL, freq_counter, mic_ampli, source_init)==ERROR_AUDIO){
				return ERROR_AUDIO;
			}
			break;

		default:
#ifdef DEBUG_AUDIO
			chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakExchange : peak_mode invalid ! \n\r");
#endif
			return ERROR_AUDIO;
	}
	return SUCCESS_AUDIO;
}

/*
 * Only fct that writes into source_init array !
 */
int16_t audioPeakWriteInit(uint8_t source_counter, uint8_t writing_mode, uint16_t freq_counter, float *mic_ampli, Source *source_init)
{
	switch(writing_mode){
		case WRITING_MODE_HIGHER:
			if(source_counter>NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
				chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakWriteInit : source_counter out of range ! \n\r");
				chprintf((BaseSequentialStream *)&SD3, "sourc_counter = %d \n\r", source_counter);
#endif
				return ERROR_AUDIO;
			}

			(&source_init[source_counter])->freq = (&source_init[source_counter-ONE])->freq;
			(&source_init[source_counter])->ampli = (&source_init[source_counter-ONE])->ampli;

			break;

		case WRITING_MODE_LOWER:
			if(source_counter>=NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
				chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakWriteInit : source_counter out of range ! \n\r");
				chprintf((BaseSequentialStream *)&SD3, "sourc_counter = %d \n\r", source_counter);
#endif
				return ERROR_AUDIO;
			}

			(&source_init[source_counter])->freq = (&source_init[source_counter+ONE])->freq;
			(&source_init[source_counter])->ampli = (&source_init[source_counter+ONE])->ampli;

			break;

		case WRITING_MODE_EQUAL:
			if(source_counter>NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
				chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakWriteInit : source_counter out of range ! \n\r");
				chprintf((BaseSequentialStream *)&SD3, "sourc_counter = %d \n\r", source_counter);
#endif
				return ERROR_AUDIO;
			}

			(&source_init[source_counter])->freq = freq_counter;
			(&source_init[source_counter])->ampli = mic_ampli[freq_counter];

			//chprintf((BaseSequentialStream *)&SD3, "audioPeakWriteInit : 				freq=%d					ampli=%f\n\r", audioConvertFreq(freq_counter), mic_ampli[freq_counter]);
			//chprintf((BaseSequentialStream *)&SD3, "audioPeakWriteInit : 	source=%d	source_init.freq=%d		source_init.ampli=%f		\n\r", source_counter, audioConvertFreq((&source_init[source_counter])->freq), (&source_init[source_counter])->ampli);
			break;
		default:
#ifdef DEBUG_AUDIO
					chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakWriteInit : writing_mode invalid ! \n\r");
#endif
					return ERROR_AUDIO;

	}
	return SUCCESS_AUDIO;
}

/*
 * Only fct that writes into global source array !
 */
int16_t audioPeakWriteSource(uint8_t source_counter, uint8_t writing_mode,  Source *source_init)
{
	if(source_counter>NB_SOURCES_MAX){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakWriteSource : source_counter out of range ! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "sourc_counter = %d \n\r", source_counter);
#endif
		return ERROR_AUDIO;
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
			return ERROR_AUDIO;
	}

	return SUCCESS_AUDIO;
}

/*
 * Bubblesort: max_freq[ZERO]=smallest_freq, max_freq[NB_SOURCES]=highest_freq
 */
int16_t audioPeakBubblesort(Source *source_init, uint8_t nb_sources_init, float *mic_ampli)
{
	uint16_t tmp_freq							= ZERO;
	float tmp_ampli								= ZERO;

	for (uint8_t source_counter=ONE; source_counter<nb_sources_init ; source_counter++){
		for (uint8_t i=ZERO; i<nb_sources_init-source_counter ; i++){
			if ((&source_init[i])->freq > (&source_init[i+ONE])->freq){
				tmp_freq = (&source_init[i])->freq;
				tmp_ampli = (&source_init[i])->ampli;
				if(audioPeakWriteInit(i, 1, WRITING_MODE_EQUAL, mic_ampli, source_init)==ERROR_AUDIO){
					return ERROR_AUDIO;
				}
				(&source_init[i+ONE])->freq = tmp_freq;
				(&source_init[i+ONE])->ampli = tmp_ampli;
			}
		}
	}
	return SUCCESS_AUDIO;
}

/**/
/*
 * Convert radian into degree
 */
uint16_t audioConvertRad(float rad)
{
	uint16_t degree = ZERO;
	degree = (int16_t) (360*(rad)/TWO_PI);
	return degree;
}

/*
 * Convert the FFT value into a real frequency [Hz]
 */
uint16_t audioConvertFreq(uint16_t freq)
{
	freq -= HALF_FFT_SIZE;
	freq = (int) (7798.8 - 15.221*freq);
	return freq;
}

/*
 * Converts Phase shift into angle
 * freq is in Hz!
 */
uint16_t audioConvertPhase(int16_t arg, uint16_t freq)
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

/*
*	Put the invoking thread into sleep until it can process the audio data
*/
void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else{
		return NULL;
	}
}




#ifdef OLD23042020
/*
 * Compares source_init and source_change
 */
uint8_t audioPeakCompareSource(Source *source1, Source *source2, uint8_t nb_sources_init)
{
	uint8_t equal	= ZERO;

	for(uint8_t source_counter=ZERO; source_counter<nb_sources_init; source_counter++){
		if((&source1[source_counter])->freq == (&source2[source_counter])->freq){
			equal++;
		}
	}

	if(equal==nb_sources_init){
		return EQUAL;
	}
	else{
		return UNEQUAL;
	}
}


/*
 * Swaps elements such that array is sorted by its amplitude
 * max_ampli[0] = smallest ampli, max_ampli[NB_SOURCES-1] = biggest ampli
 */
void audioPeakShift(uint8_t nb_shifts)
{
	if((nb_shifts>ZERO) && (nb_shifts<NB_SOURCES_MAX)){
		for(uint8_t i=ZERO; i<nb_shifts; i++){
			source[i].ampli = source[i+ONE].ampli;
			source[i].freq = source[i+ONE].freq;
		}
	}
}

/*Old code for audioPeakExchange*/
if(nb_sources<NB_SOURCES_MAX){
		for(source_counter=nb_sources; source_counter>source_exchange; source_counter--){
			if(audioPeakWrite(source_counter, 0, freq_counter, mic_ampli, mode)==ERROR_AUDIO){
				return ERROR_AUDIO;
			}
		}
		if(audioPeakWrite(source_exchange, 2, freq_counter, mic_ampli, mode)==ERROR_AUDIO){
			return ERROR_AUDIO;
		}
		nb_sources++;
	}
	else if(nb_sources==NB_SOURCES_MAX){
		source_exchange--;
		for(source_counter=ZERO; source_counter<source_exchange; source_counter++){
			if(audioPeakWrite(source_counter, 1, freq_counter, mic_ampli, mode)==ERROR_AUDIO){
				return ERROR_AUDIO;
			}

		}
		if(audioPeakWrite(source_exchange, 2, freq_counter, mic_ampli, mode)==ERROR_AUDIO){
			return ERROR_AUDIO;
		}
	}
	else{
#ifdef DEBUG_AUDIO
			chprintf((BaseSequentialStream *)&SD3, "ERROR audioPeakExchange : nb_sources out of range ! \n\r");
			chprintf((BaseSequentialStream *)&SD3, "nb_sources = %d \n\r", nb_sources);
#endif
			return ERROR_AUDIO;

}

/*Old code for audio calculate Peak*/
for(uint16_t freq_counter=HALF_FFT_SIZE; freq_counter<FFT_SIZE; freq_counter++){
	source_exchange=ZERO;
	source_counter=ZERO;
//			for(source_counter=ZERO; source_counter<NB_SOURCES; source_counter++){
//				if(mic_ampli[freq_counter]>source[source_counter].ampli){
//					source_exchange++;
//				}
//			}
	while((source_counter<NB_SOURCES) && (mic_ampli[freq_counter]>source[source_counter].ampli)){
		source_exchange++;
		source_counter++;
	}
	if(source_exchange>ZERO){
		for(source_counter=ZERO; source_counter<NB_SOURCES; source_counter++){
			if((freq_counter-source[source_counter].freq)<FREQ_THD){
				source_exchange = ZERO;
				if(mic_ampli[freq_counter]>source[source_counter].ampli){
					source[source_counter].ampli = mic_ampli[freq_counter];
					source[source_counter].freq = freq_counter;
				}
				//break;
			}

		}
	}

	if(source_exchange>ZERO){
		audioPeakShift(source_exchange-ONE);
		source[source_exchange-ONE].ampli = mic_ampli[freq_counter];
		source[source_exchange-ONE].freq = freq_counter;
	}

	for (source_counter=ONE; source_counter< NB_SOURCES ; source_counter++){
	  for (uint8_t i=ZERO; i<NB_SOURCES-source_counter ; i++){
		  if (source[i].ampli > source[i+ONE].ampli){
			  tmp = source[i].ampli;
			  tmp2 = source[i].freq;
			  source[i].ampli = source[i+ONE].ampli;
			  source[i].freq = source[i+ONE].freq;
			  source[i+ONE].ampli = tmp;
			  source[i+ONE].freq = tmp2;
		  }
	  }
   }

}
/*for(uint16_t freq_counter=HALF_FFT_SIZE; freq_counter<FFT_SIZE; freq_counter++){
	source_exchange=ZERO;
	for(source_counter=ZERO; source_counter<NB_SOURCES; source_counter++){
		if(mic_ampli[freq_counter]>source[source_counter].ampli){
			source_exchange++;
		}
	}
	if(source_exchange>ZERO){
		audioPeakShift(source_exchange-ONE);
		source[source_exchange-ONE].ampli = mic_ampli[freq_counter];
		source[source_exchange-ONE].freq = freq_counter;
	}
}*/



int16_t audioDeterminePhase(float *mic_data1, float *mic_data2, float *mic_ampli1, float *mic_ampli2, uint16_t *freq_index, uint8_t source_index)
{

	/*Variables*/
	float ampli_max_value1			=ZERO;
	float ampli_max_value2			=ZERO;
	uint16_t ampli_max_index1		=ZERO;
	uint16_t ampli_max_index2		=ZERO;
	float phase1						=ZERO;								//in rad [-pi,+pi]
	float phase2						=ZERO;								//in rad [-pi,+pi]
	int16_t phase_dif				=ZERO;								//in degrees [-180°,+180°]
#ifdef DEBUG_AUDIO
	int16_t phase1_deg				=ZERO;								//in degrees [-180°,+180°]
	int16_t phase2_deg				=ZERO;								//in degrees [-180°,+180°]
#endif


	/*Calculate FFT and it's amplitude*/
	doFFT_optimized(FFT_SIZE, mic_data1);
	doFFT_optimized(FFT_SIZE, mic_data2);
	arm_cmplx_mag_f32(mic_data1, mic_ampli1, FFT_SIZE);
	arm_cmplx_mag_f32(mic_data2, mic_ampli2, FFT_SIZE);

	/*Find maximal amplitude of mic1 and mic2*/
	for(uint16_t freq_counter=HALF_FFT_SIZE; freq_counter<FFT_SIZE; freq_counter++){
		if(mic_ampli1[freq_counter]>ampli_max_value1){
			ampli_max_value1=mic_ampli1[freq_counter];
			ampli_max_index1=freq_counter;
		}
		if(mic_ampli2[freq_counter]>ampli_max_value2){
			ampli_max_value2=mic_ampli1[freq_counter];
			ampli_max_index2=freq_counter;
		}
	}

	/*Error: Peak freq. of mic1 and mic2 are not the same*/
	if(ampli_max_index1!=ampli_max_index2){
#ifdef DEBUG_AUDIO
		ampli_max_index1 = audioConvertFreq(ampli_max_index1);
		ampli_max_index2 = audioConvertFreq(ampli_max_index2);
		chprintf((BaseSequentialStream *)&SD3, "Error: 		Max value freq are not equal! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Max value1 (freq/ampli) : 	%d			%f \n\r", ampli_max_index1, ampli_max_value1);
		chprintf((BaseSequentialStream *)&SD3, "Max value2 (freq/ampli) : 	%d			%f \n\r", ampli_max_index2, ampli_max_value2);
#endif
		return ERROR_AUDIO;
	}

	/*Calculate phase shift between the signal of mic1 and mic2; atan2f(float y, float x) returns float arctan(y/x) in rad [-pi,+pi]*/
	phase1 = atan2f(mic_data1[ ((CMPX_VAL*ampli_max_index1)+ONE) ], mic_data1[ (CMPX_VAL*ampli_max_index1) ]);
	phase2 = atan2f(mic_data2[ ((CMPX_VAL*ampli_max_index2)+ONE) ], mic_data2[ (CMPX_VAL*ampli_max_index2) ]);
	phase_dif = audioConvertRad(phase1-phase2);

	/*Error: Phase out of range; Outside of [-pi,+pi]*/
	if(phase1>PI || phase1<(-PI) || phase2>PI || phase2<(-PI)){
#ifdef DEBUG_AUDIO
		phase1_deg = audioConvertRad(phase1);
		phase2_deg = audioConvertRad(phase2);
		chprintf((BaseSequentialStream *)&SD3, "Error: 		phase1 or Ar2 out of range! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Phase1 (rad/deg): 		%f 		%d\n\r", phase1, phase1_deg);
		chprintf((BaseSequentialStream *)&SD3, "Phase2 (rad/deg): 		%f 		%d\n\r", phase2, phase2_deg);
#endif
		return ERROR_AUDIO;
	}

	/*Error: Arg dif out of range; Max arg dif for all freq. below 1200Hz*/
	if(phase_dif>PHASE_DIF_LIMIT || phase_dif<(-PHASE_DIF_LIMIT)){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "Error: 		Phase dif out of range! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Phase dif (deg):		%d\n\r", phase_dif);
#endif
		return ERROR_AUDIO;
	}

	/*Conversion into Hz*/
	ampli_max_index1 = audioConvertFreq(ampli_max_index1);
	*freq_index = ampli_max_index1;

	return phase_dif;
}
#endif
