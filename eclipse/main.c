#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

#include <main.h>

#include <travelController.h>
#include <comms.h>

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

#define NB_BYTE_PER_CMPX_VAL			2


static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void)
{
	halInit(); //
	chSysInit();

	comms_start();

	//TESTPING test travelController functions!
	travCtrl_testAll();

    mpu_init();

    //starts timer 12
    timer12_start();

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it

    static float mic_data_right[NB_BYTE_PER_CMPX_VAL*FFT_SIZE];
    static float mic_data_left[NB_BYTE_PER_CMPX_VAL*FFT_SIZE];
    static float mic_ampli_right[FFT_SIZE];
    static float mic_ampli_left[FFT_SIZE];

    /* SEND_FROM_MIC */
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);


    /* Infinite loop. */
    while (1) {

        //waits until a result must be sent to the computer
    		wait_send_to_computer();

        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_CMPLX_INPUT), mic_data_left, NB_BYTE_PER_CMPX_VAL*FFT_SIZE);
        arm_copy_f32(get_audio_buffer_ptr(RIGHT_CMPLX_INPUT), mic_data_right, NB_BYTE_PER_CMPX_VAL*FFT_SIZE);

        audioAnalyseDirection(mic_data_left, mic_data_right, mic_ampli_left, mic_ampli_right);
    }
}

/*===========================================================================*/
/* Something to protect agains something...                                  */
/*===========================================================================*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
