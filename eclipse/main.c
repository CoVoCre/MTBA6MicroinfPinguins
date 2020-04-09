#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <main.h>



#include <travelController.h>
#include <comms.h>


bool destReached = false;

travCtrl_dirAngleCb_t updateAngle;

void destReachedCB(void){
	destReached = true;
}

int16_t newAngle = 20;

int main(void)
{
	halInit(); //
	chSysInit();

	comms_start();

	chprintf((BaseSequentialStream *)&SD3,"Beginning of main\n\r");

	updateAngle = travCtrl_init(destReachedCB);
	updateAngle(newAngle);
//	travCtrl_testAll();
	while(1){
		//chprintf((BaseSequentialStream *)&SD3, "In MAIN\n\r");
		chThdSleepMilliseconds(100);
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
