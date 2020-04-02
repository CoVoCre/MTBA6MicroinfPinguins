#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <main.h>

//#include <travelController.h>


int main(void)
{
//	travCtrl_testAll();
	while(1){
	
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
