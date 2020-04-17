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
#define AMPLI_THD						5000						//Threshold for peak-ampli
#define	FREQ_THD							2						//Threshold corresponding to 30Hz

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


/*Semaphore*/
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);


/*Static Variables*/

/*Audio buffer: 2*FFT_SIZE because arrays contain complex numbers (real + imaginary)*/
static float micLeft_cmplx_input[CMPX_VAL*FFT_SIZE];
static float micRight_cmplx_input[CMPX_VAL*FFT_SIZE];
static float micFront_cmplx_input[CMPX_VAL*FFT_SIZE];
static float micBack_cmplx_input[CMPX_VAL*FFT_SIZE];

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
 * Determines the peak freq of all sources
 */
void audioDetermineSource(float *mic_data, float *mic_ampli, uint16_t *source)
{
	doFFT_optimized(FFT_SIZE, mic_data);
	arm_cmplx_mag_f32(mic_data, mic_ampli, FFT_SIZE);

	for(uint8_t source_index=ZERO; source_index<NB_SOURCES; source_index++){
		source[source_index] = audioCalculatePeak(mic_ampli, source_index);
	}
}

/*
 * Calculate angle of sound direction
 */
int16_t audioAnalyseDirection(float *mic_data_left, float *mic_data_right, float *mic_data_back, float *mic_data_front,
							float *mic_ampli_left, float *mic_ampli_right, float *mic_ampli_back, float *mic_ampli_front, uint8_t source_index)
{
	uint16_t freq_index						= ZERO;
	int16_t arg_dif_left_right				= ZERO;
	int16_t arg_dif_back_front				= ZERO;
	int16_t arg_dif							= ZERO;
	static uint8_t ema_counter				= ZERO;
	static int16_t ema_arg_dif				= ZERO;

	/*Calculate the angle shift with respect to the central axe of the robot*/
	arg_dif_left_right = audioCalculatePhase(mic_data_left, mic_data_right, mic_ampli_left, mic_ampli_right, &freq_index, source_index);
	arg_dif_back_front = audioCalculatePhase(mic_data_back, mic_data_front, mic_ampli_back, mic_ampli_front, &freq_index, source_index) ;

	/*Verify if there was an error in audioCalculatePhase*/
	if(arg_dif_left_right==ERROR_AUDIO || arg_dif_back_front==ERROR_AUDIO){
		return ERROR_AUDIO;
	}

	/*Verify if there was an error in audioCalculatePhase*/
	if(freq_index==ZERO){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "Error : freq is NULL\n\r");
#endif
		return ERROR_AUDIO;
	}

	/*Convert phase shift into angle*/
	arg_dif_left_right = audioConvertPhase(arg_dif_left_right, freq_index);
	arg_dif_back_front = audioConvertPhase(arg_dif_back_front, freq_index);

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
	if(ema_counter==ZERO){
		ema_arg_dif = arg_dif;
		ema_counter++;
	}
	else if(((ema_arg_dif<-DEG90) && (arg_dif>DEG90)) || ((ema_arg_dif>DEG90) && (arg_dif<-DEG90))){				//Jump from 180° to -180° or inverse
		ema_arg_dif = arg_dif;
	}
	else{
		ema_arg_dif = (int16_t) (arg_dif*EMA_WEIGHT + ema_arg_dif*(ONE-EMA_WEIGHT));
	}

	return ema_arg_dif;
}

/*
 * Calculate phase difference between two mic.
 */
int16_t audioCalculatePhase(float *mic_data1, float *mic_data2, float *mic_ampli1, float *mic_ampli2, uint16_t *freq_index, uint8_t source_index)
{

	/*Variables*/
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

	ampli_max_index1 = audioCalculatePeak(mic_ampli1, source_index);
	ampli_max_index2 = audioCalculatePeak(mic_ampli2, source_index);

	/*Error: Peak freq. of mic1 and mic2 are not the same*/
	if(ampli_max_index1!=ampli_max_index2){
#ifdef DEBUG_AUDIO
		ampli_max_index1 = audioConvertFreq(ampli_max_index1);
		ampli_max_index2 = audioConvertFreq(ampli_max_index2);
		chprintf((BaseSequentialStream *)&SD3, "Error: 		Max value freq are not equal! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Source %d :		Max freq1 : 	%d			 \n\r", source_index, ampli_max_index1);
		chprintf((BaseSequentialStream *)&SD3, "Source %d :		Max freq2 : 	%d			\n\r", source_index, ampli_max_index2);
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

	/*Conversion into Hz*/
	ampli_max_index1 = audioConvertFreq(ampli_max_index1);
	*freq_index = ampli_max_index1;

	return phase_dif;
}

/*
 * Calculates NB_SOURCES peak values and sorts them after freq
 * Returns peak freq of source_index; source_index=ZERO: lowest_freq, source_index=NB_SOURCES-ONE: highest_freq
 */
uint16_t audioCalculatePeak(float *mic_ampli, uint8_t source_index)
{
	uint8_t source_counter				= ZERO;
	uint16_t tmp							= ZERO;
	uint16_t max_freq[NB_SOURCES] 		= {0};
	float max_ampli[NB_SOURCES] 			= {0};

	/*Find peak freq and sort arrays by ampli: peak[0] = smallest ampli, peak[NB_SOURCES-1] = biggest ampli*/
	for(uint16_t freq_counter=HALF_FFT_SIZE; freq_counter<FFT_SIZE; freq_counter++){
		source_counter = ZERO;
		while((source_counter<NB_SOURCES) && (mic_ampli[freq_counter]>max_ampli[source_counter])){
			audioPeakSwap(source_counter, max_freq, max_ampli);
			source_counter++;
		}
		if(source_counter > ZERO){
			max_ampli[source_counter-ONE] = mic_ampli[freq_counter];
			max_freq[source_counter-ONE] = freq_counter;
		}
	}

	/*Bubblesort: max_freq[ZERO]=smallest_freq, max_freq[NB_SOURCES]=highest_freq*/
   for (source_counter=ONE; source_counter< NB_SOURCES ; source_counter++){
	  for (uint8_t i=ZERO; i<NB_SOURCES-source_counter ; i++){
		  if (max_freq[i] > max_freq[i+ONE]){
			  tmp = max_freq[i];
			  max_freq[i] = max_freq[i+ONE];
			  max_freq[i+1] = tmp;
		  }
	  }
   }

	/*Error: Peak freq. out of range [150Hz,1200Hz], peak[i].freq is not in Hz!*/
	if((max_freq[source_index]>FREQ_MIN) || (max_freq[source_index]<FREQ_MAX)){						//Inverse logic because freq is not in Hz!
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : 	Max freq out of range ! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Source %d :			Peak freq = %d \n\r", source_index, audioConvertFreq(max_freq[source_index]));
#endif
		return ERROR_AUDIO;
	}

   	/*Error: Two peak freq are two close, difference<30Hz*/
   	for(source_counter=ZERO; source_counter<NB_SOURCES; source_counter++){
   		for(uint8_t i=ZERO; i<NB_SOURCES; i++){
   			if((i!=source_counter) && (abs(max_freq[source_counter]-max_freq[i])<FREQ_THD)){
#ifdef DEBUG_AUDIO
   				chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : Max freq too close ! \n\r");
   				chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max freq = %d \n\r", source_counter, audioConvertFreq(max_freq[source_counter]));
   				chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max freq = %d \n\r", i, audioConvertFreq(max_freq[i]));
#endif
   				return ERROR_AUDIO;
   			}
   		}
   	}

	/*Error: Peak ampli is too low*/
	if(max_ampli[source_index]<AMPLI_THD){
#ifdef DEBUG_AUDIO
		chprintf((BaseSequentialStream *)&SD3, "ERROR audioCalcPeak : Max ampli too low ! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Source %d :	Max ampli = %f \n\r", source_index, max_ampli[source_index]);
#endif
		return ERROR_AUDIO;
	}

	return max_freq[source_index];
}

/*
 * Swaps elements such that array is sorted by its amplitude
 * max_ampli[0] = smallest ampli, max_ampli[NB_SOURCES-1] = biggest ampli
 */
void audioPeakSwap(uint8_t swaps, uint16_t *max_freq, float *max_ampli){
	if((swaps<NB_SOURCES) && (swaps>ZERO)){
		max_ampli[swaps-ONE] = max_ampli[swaps];
		max_freq[swaps-ONE] = max_freq[swaps];
	}
}

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
 */
uint16_t audioConvertPhase(int16_t arg, uint16_t freq){
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




#ifdef OLD17042020
int16_t audioCalculatePhase(float *mic_data1, float *mic_data2, float *mic_ampli1, float *mic_ampli2, uint16_t *freq_index, uint8_t source_index)
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
