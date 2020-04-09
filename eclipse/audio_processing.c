#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define NB_BYTE_PER_CMPX_VAL				2
#define NB_OF_MICROPHONES				4
#define ZERO								0
#define ONE								1
#define TWO_PI							6.2832
#define HALF_FFT_SIZE					512
#define ARG_DIF_LIMIT					1.3189					//Max arg dif for all freq. below 1200Hz
#define EMA_WEIGHT						0.1						//range [0,1], if smaller past results have more weight

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t samples_gathered = 0;
	//static uint8_t send_counter = ZERO;
	uint16_t sample_counter=0;

	while(sample_counter<num_samples){
		if(samples_gathered<FFT_SIZE){
			micRight_cmplx_input[2*samples_gathered] = data[sample_counter];
			micRight_cmplx_input[2*samples_gathered+1] = ZERO;
			micLeft_cmplx_input[2*samples_gathered] = data[sample_counter+1];
			micLeft_cmplx_input[2*samples_gathered+1] = ZERO;
			micBack_cmplx_input[2*samples_gathered] = data[sample_counter+2];
			micBack_cmplx_input[2*samples_gathered+1] = ZERO;
			micFront_cmplx_input[2*samples_gathered] = data[sample_counter+3];
			micFront_cmplx_input[2*samples_gathered+1] = ZERO;
			sample_counter+=4;
			samples_gathered++;
		}
		else{
			samples_gathered=0;
			chBSemSignal(&sendToComputer_sem);

			/*Treat and send only every 10 measurement*/
			/*if(send_counter<10){
				send_counter++;
			}
			else{
				doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
				arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
				SendFloatToComputer((BaseSequentialStream *) &SD3, micLeft_output, FFT_SIZE);

				send_counter=ZERO;
				chBSemSignal(&sendToComputer_sem);
			}*/
		}
	}
}

void audioAnalyseDirection(float *mic_data1, float *mic_data2, float *mic_ampli1, float *mic_ampli2)
{
	/*Sending sound in the time domain to the computer*/
	/*static float mic_sin1[FFT_SIZE];
	static float mic_sin2[FFT_SIZE];

	for(uint16_t i=ZERO; i<FFT_SIZE;i++){
		mic_sin1[i]=mic_data1[NB_BYTE_PER_CMPX_VAL*i];
		mic_sin2[i]=mic_data2[NB_BYTE_PER_CMPX_VAL*i];
	}
	SendFloatToComputer((BaseSequentialStream *) &SD3, mic_sin2, FFT_SIZE);
	SendFloatToComputer((BaseSequentialStream *) &SDU1, mic_sin1, FFT_SIZE);*/

	/*Variables*/
	float ampli_max_value1			=ZERO;
	uint16_t ampli_max_index1		=ZERO;
	float ampli_max_value2			=ZERO;
	uint16_t ampli_max_index2		=ZERO;
	float arg_max_ampli1				=ZERO;								//in rad [-pi,+pi]
	float arg_max_ampli2				=ZERO;								//in rad [-pi,+pi]
	float arg_dif					=ZERO;								//in rad [-pi,+pi]
	int16_t arg1_deg					=ZERO;								//in degrees [-180°,+180°]
	int16_t arg2_deg					=ZERO;								//in degrees [-180°,+180°]
	int16_t arg_dif_deg				=ZERO;								//in degrees [-180°,+180°]
	static uint8_t ema_counter		=ZERO;
	static float ema_arg_dif			=ZERO;
	int16_t ema_arg_dif_deg			=ZERO;

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
		/*Conversion into Hz*/
		ampli_max_index1 = audioConvertFreq(ampli_max_index1);
		ampli_max_index2 = audioConvertFreq(ampli_max_index2);

		chprintf((BaseSequentialStream *)&SD3, "Error: 		Max value freq are not equal! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Max value1 (freq/ampli) : 	%d			%f \n\r", ampli_max_index1, ampli_max_value1);
		chprintf((BaseSequentialStream *)&SD3, "Max value2 (freq/ampli) : 	%d			%f \n\r\n\r", ampli_max_index2, ampli_max_value2);
		return;
	}

	/*Calculate angle shift between the signal of mic1 and mic2; atan2f(float y, float x) returns float arctan(y/x) in rad [-pi,+pi]*/
	arg_max_ampli1 = atan2f(mic_data1[ ((NB_BYTE_PER_CMPX_VAL*ampli_max_index1)+ONE) ], mic_data1[ (NB_BYTE_PER_CMPX_VAL*ampli_max_index1) ]);
	arg_max_ampli2 = atan2f(mic_data2[ ((NB_BYTE_PER_CMPX_VAL*ampli_max_index2)+ONE) ], mic_data2[ (NB_BYTE_PER_CMPX_VAL*ampli_max_index2) ]);
	arg_dif = arg_max_ampli1 - arg_max_ampli2;
	arg1_deg= (int16_t) (360*(arg_max_ampli1)/TWO_PI);
	arg2_deg= (int16_t) (360*(arg_max_ampli2)/TWO_PI);
	arg_dif_deg= (int16_t) (360*(arg_dif)/TWO_PI);

	/*Error: Angle out of range; Outside of [-pi,+pi]*/
	if(arg_max_ampli1>PI || arg_max_ampli1<(-PI) || arg_max_ampli2>PI || arg_max_ampli2<(-PI)){
		chprintf((BaseSequentialStream *)&SD3, "Error: 		Arg1 or Ar2 out of range! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Arg1 (rad/deg): 		%f 		%d\n\r", arg_max_ampli1, arg1_deg);
		chprintf((BaseSequentialStream *)&SD3, "Arg2 (rad/deg): 		%f 		%d\n\r\n\r", arg_max_ampli2, arg2_deg);
		return;
	}

	/*Error: Arg dif out of range; Max arg dif for all freq. below 1200Hz*/
	if(arg_dif>ARG_DIF_LIMIT || arg_dif<(-ARG_DIF_LIMIT)){
		chprintf((BaseSequentialStream *)&SD3, "Error: 		Arg dif out of range! \n\r");
		chprintf((BaseSequentialStream *)&SD3, "Arg dif (rad/deg): 		%f 		%d\n\r\n\r", arg_dif, arg_dif_deg);
		return;
	}

	/*Exponential Moving Average (EMA); EMA_WEIGHT range: [0,1], if smaller past results have more weight*/
	if(ema_counter==ZERO){
		ema_arg_dif = arg_dif;
		ema_counter++;
	}
	else{
		ema_arg_dif = arg_dif*EMA_WEIGHT + ema_arg_dif*(ONE-EMA_WEIGHT);
	}
	ema_arg_dif_deg= (int16_t) (360*ema_arg_dif/TWO_PI);

	//chprintf((BaseSequentialStream *)&SD3, "Max Value1: 			%f + 	%f i \n\r", mic_data1[(NB_BYTE_PER_CMPX_VAL*ampli_max_index1)], mic_data1[(NB_BYTE_PER_CMPX_VAL*ampli_max_index1)+ONE]);
	//chprintf((BaseSequentialStream *)&SD3, "Max Value2: 			%f + 	%f i \n\r", mic_data2[(NB_BYTE_PER_CMPX_VAL*ampli_max_index2)], mic_data2[(NB_BYTE_PER_CMPX_VAL*ampli_max_index2)+ONE]);
	chprintf((BaseSequentialStream *)&SD3, "Arg1 (rad/deg): 		%f 		%d\n\r", arg_max_ampli1, arg1_deg);
	chprintf((BaseSequentialStream *)&SD3, "Arg2 (rad/deg): 		%f 		%d\n\r", arg_max_ampli2, arg2_deg);
	chprintf((BaseSequentialStream *)&SD3, "Arg dif (rad/deg): 		%f 		%d\n\r", arg_dif, arg_dif_deg);
	chprintf((BaseSequentialStream *)&SD3, "EMA arg dif (rad/deg): 		%f 		%d\n\r", ema_arg_dif, ema_arg_dif_deg);

	/*Conversion into Hz*/
	ampli_max_index1 = audioConvertFreq(ampli_max_index1);
	ampli_max_index2 = audioConvertFreq(ampli_max_index2);

	//chprintf((BaseSequentialStream *)&SD3, "Max value1 (freq/ampli) : 	%d			%f \n\r", ampli_max_index1, ampli_max_value1);
	//chprintf((BaseSequentialStream *)&SD3, "Max value2 (freq/ampli) : 	%d			%f \n\r\n\r", ampli_max_index2, ampli_max_value2);




	/*chprintf((BaseSequentialStream *)&SD3, "Ampli max index: 		%d		Ampli max value: 	%f \n\r", ampli_max_index, ampli_max_value);
	chprintf((BaseSequentialStream *)&SD3, "Ampli max index-2: 	%d		Ampli max value-2:	%f \n\r", (ampli_max_index-2), mic_ampli_left[(ampli_max_index-20)]);
	chprintf((BaseSequentialStream *)&SD3, "Ampli max index-1: 	%d		Ampli max value-1:	%f \n\r", (ampli_max_index-1), mic_ampli_left[(ampli_max_index-1)]);
	chprintf((BaseSequentialStream *)&SD3, "Ampli max index+1: 	%d		Ampli max value+1:	%f \n\r", (ampli_max_index+1), mic_ampli_left[(ampli_max_index+1)]);
	chprintf((BaseSequentialStream *)&SD3, "Ampli max index+2: 	%d		Ampli max value+2:	%f \n\r", (ampli_max_index+2), mic_ampli_left[(ampli_max_index+20)]);
	*/


	/*if(counter<5){
		arg_max_ampli_left = atan2f(mic_data_left[ (NB_BYTE_PER_CMPX_VAL*(ampli_max_index)+ONE) ], mic_data_left[ (NB_BYTE_PER_CMPX_VAL*(ampli_max_index)) ]);
		arg_max_ampli_right = atan2f(mic_data_right[ (NB_BYTE_PER_CMPX_VAL*(ampli_max_index)+ONE) ], mic_data_right[ (NB_BYTE_PER_CMPX_VAL*(ampli_max_index)) ]);
		arg_dif = arg_max_ampli_left - arg_max_ampli_right;
		arg_tot_dif += arg_dif;
		chprintf((BaseSequentialStream *)&SD3, "Arg dif: 	%f \n\r", arg_dif);
		counter++;
	}
	else{
		arg_max_ampli_left = atan2f(mic_data_left[ (NB_BYTE_PER_CMPX_VAL*(ampli_max_index)+ONE) ], mic_data_left[ (NB_BYTE_PER_CMPX_VAL*(ampli_max_index)) ]);
		arg_max_ampli_right = atan2f(mic_data_right[ (NB_BYTE_PER_CMPX_VAL*(ampli_max_index)+ONE) ], mic_data_right[ (NB_BYTE_PER_CMPX_VAL*(ampli_max_index)) ]);
		arg_dif = (arg_max_ampli_left-arg_max_ampli_right);
		arg_tot_dif += arg_dif;
		arg_tot_dif = arg_tot_dif*0.14286;
		counter=ZERO;
		chprintf((BaseSequentialStream *)&SD3, "Arg dif: 	%f \n\r", arg_dif);
		chprintf((BaseSequentialStream *)&SD3, "Arg tot dif: 	%f \n\r", arg_tot_dif);
	}*/



	//ampli_max_index = audioConvertFreq(ampli_max_index);
	//arg_difference= (int8_t) (360*(arg_max_ampli_left-arg_max_ampli_right)/TWO_PI);
	//fft_shift=(HALF_FFT_SIZE/(TWO_PI*ampli_max_index))*(arg_max_ampli_left-arg_max_ampli_right);

	//SendFloatToComputer((BaseSequentialStream *) &SD3, mic_ampli_left, FFT_SIZE);
	//chprintf((BaseSequentialStream *)&SD3, "Max Ampli Value : %d \n\rMax Ampli index : %d \n\rArg left : %f \n\rArg right : %f \n\rArg dif : %d \n\rShift : %f \n\r\n\r", ampli_max_value, ampli_max_index, arg_max_ampli_left, arg_max_ampli_right, arg_difference, fft_shift);
}

/*Converts the FFT value into a real frequency [Hz]*/
uint16_t audioConvertFreq(uint16_t freq)
{
	freq -= HALF_FFT_SIZE;
	freq = (int) (3902 - 7.622*freq);
	return freq;
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

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
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
