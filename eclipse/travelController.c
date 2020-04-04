/*
 * travelController.c
 *
 *  Created on: Apr 2, 2020
 *      Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * and stops when an obstacle/the objective is reached (detection with proximity sensor).
 * Functions prefix for this file: travCtrl_
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h> //for threads

#include <motors.h>


#include <travelController.h>

// From motors.h library functions need steps/s max speed 1100steps/s
// here is max +/-126 because not much precision needed, will see what max speed
// we want and coeff between steps/s and variable increments should be
// and also if a linear relation is OK
#define MOT_MAX_NEEDED_SPS 500
#define MOT_SPEED_RANGE 127
#define MOT_STEPSPS_TO_VAR_RANGE(STEPS_S) ( ((MOT_SPEED_RANGE*STEPS_S) /MOT_MAX_NEEDED_SPS) < MOT_SPEED_RANGE ?	\
											((MOT_SPEED_RANGE*STEPS_S) /MOT_MAX_NEEDED_SPS) : MOT_SPEED_RANGE )

#define MOT_MAX_ANGLE_TO_CORRECT 90	// this will be the max angle in ° that the correction will still change
#define MOT_MAX_DIFF_SPS_FOR_CORRECTION 300
#define MOT_CORRECTION_EXPONENT 2.5 //can range from 1 (no less than linear) to technically anything, and with decimals

#define MOT_CONTROLLER_PERIOD 10 //in ms, will be the interval at which controller thread will re-adjust control
#define MOT_CONTROLLER_WORKING_AREA_SIZE 128 //128 because it should be enough !



int16_t destAngle = 0; //from -179 to +180
uint8_t destDistance = 0; // from 0 to 255 with 255 max range of proximity sensor
int8_t leftMotSpeed = 0; //from -126 to +126, use MOT_STEPSPS_TO_VAR_RANGE(STEPS_S) to use steps per sec
int8_t rightMotSpeed = 0; //from -126 to +126, use MOT_STEPSPS_TO_VAR_RANGE(STEPS_S) to use steps per sec
thread_t *motCtrlThread;

travCtrl_destReached destReachedFctToCall;

/*===========================================================================*/
/* Internal functions definitions             */
/*===========================================================================*/
void dirAngleCb(int16_t newDestAngle);


/*===========================================================================*/
/* Private functions              */
/*===========================================================================*/

/* Working area for the motor controller. */
static THD_WORKING_AREA(waMotControllerThd, MOT_CONTROLLER_WORKING_AREA_SIZE);
/* Motor controller thread. */
static THD_FUNCTION(MotControllerThd, arg) {
    systime_t time;
    static bool motCtrShldContinue = true;
	while (motCtrShldContinue) {
		time = chVTGetSystemTime();
		// should do things here or call a function that does !
		motCtrShldContinue = proxDistanceUpdate();
		motControllerUpdate();
		chThdSleepUntilWindowed(time, time + MS2ST(MOT_CONTROLLER_PERIOD));
	}
	chThdExit(true);
}

/**
 * @brief   Updates the measured distance to object
 * @return  false if destination is not reached 0, true otherwise
*/
bool proxDistanceUpdate(void){
	bool destNotReached = true;
	// TODO things here
	return destNotReached;
}

/**
 * @brief   Updates the speeds of the motors based on distance and angle
*/
void motControllerUpdate(void){
	// TODO first speed differential based on angle

	// TODO then update speed based on distance

}

/**
 * @brief   function to be called when the destination is reached according to prox sensor
*/
void destinationReached(void){
	// set termination flag to true in motCtrlThread
	chThdTerminate(motCtrlThread);
	chThdWait(motCtrlThread); // wait until thread has effectively stopped
	destReachedFctToCall();
}

/**
 * @brief   Callback fct given to exterior for when the angle needs updating.
 * @parameter [in] newDestAngle new angle direction between -179° and 180°
*/
void dirAngleCb(int16_t newDestAngle){
	destAngle = newDestAngle;
}

/*===========================================================================*/
/* Public functions for setting/getting internal parameters             */
/*===========================================================================*/

travCtrl_dirAngleCb_t travCtrl_init(travCtrl_destReached destReachedCallback){
	// start things, motor, proximity, thread for updating control params etc
	//inits the motors
	motors_init();

	destAngle = 0;
	destDistance = 0;

	destReachedFctToCall = destReachedCallback;

	//start of controller thread here
	motCtrlThread = 	chThdCreateStatic(waMotControllerThd, sizeof(waMotControllerThd), NORMALPRIO, MotControllerThd, NULL);

	return &dirAngleCb;
}

/*===========================================================================*/
/* Functions for testing              */
/*===========================================================================*/
//void travCtrl_testAll(void){
//	int16_t testNewAngle = 10;
//	//travCtrl_dirAngleCb_t updateAngleCB = travCtrl_init();
//	updateAngleCB(testNewAngle);
//}
