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
#include <sensors/VL53L0X/VL53L0X.h>

#include <travelController.h>

// From motors.h library functions need steps/s max speed 1100steps/s (MOTOR_SPEED_LIMIT) but for us might need less
// @warning do not set speed above MOTOR_SPEED_LIMIT - MOT_MAX_DIFF_SPS_FOR_CORRECTION otherwise it couldn't turn !
#define MOT_MAX_NEEDED_SPS 500
#define MAX_DISTANCE_VALUE_MM 500 //how far in mm should robot start to slow
#define STOP_DISTANCE_VALUE_MM 30 //how far in mm should robot stop

#define MOT_MAX_ANGLE_TO_CORRECT 100	// this will be the max angle in ° that the correction will still change
#define MOT_MAX_DIFF_SPS_FOR_CORRECTION 300
//#define MOT_CORRECTION_EXPONENT 2.5 //can range from 1 (no less than linear) to technically anything, and with decimals
#define MOT_KP_DIFF 1	//needs >=0 value
#define MOT_KI_DIFF 0.5 //needs >=0 value
#define MOT_KI_N_ANGLES 5

#define MOT_CONTROLLER_PERIOD 10 //in ms, will be the interval at which controller thread will re-adjust control
#define MOT_CONTROLLER_WORKING_AREA_SIZE 128 //128 because it should be enough !

#define IR_FRONT_RIGHT 0 //IR1 so sensor number 0
#define IR_FRONT_LEFT 7 //IR8 so sensor number 7
#define IR_CALIB_0_DISTANCE -3700 //from http://www.e-puck.org/index.php?option=com_content&view=article&id=22&Itemid=13
#define IR_CALIB_MAX_RANGE -750 //same source, actually only somewhat linear bewteen -3700 and -1000

int16_t destAngle = 0; //from -179 to +180
int16_t lastNAngles[MOT_KI_N_ANGLES] = {0}; //set all to 0
uint16_t destDistanceMM = 0;
int rightMotSpeed = 0; //from -126 to +126, it is an int as in motors.h
int leftMotSpeed = 0; //from -126 to +126, it is an int as in motors.h
thread_t *motCtrlThread; // pointer to motor controller thread if needed to stop it TODOPING maybe remove if not necessary anymore

travCtrl_destReached destReachedFctToCall;

/*===========================================================================*/
/* Internal functions definitions             */
/*===========================================================================*/
void dirAngleCb(int16_t newDestAngle);
bool proxDistanceUpdate(void);
void motControllerUpdate(void);

/*===========================================================================*/
/* Private functions              */
/*===========================================================================*/

/* Working area for the motor controller. */
static THD_WORKING_AREA(waMotControllerThd, MOT_CONTROLLER_WORKING_AREA_SIZE);
/* Motor controller thread. */
static THD_FUNCTION(MotControllerThd, arg) {
	(void)arg; // silence warning about unused argument
    systime_t time;
    static bool motCtrShldContinue = true;
	while (motCtrShldContinue) {
		time = chVTGetSystemTime();
		// TODOPING should do things here or call a function that does !
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
	bool destIsNotReached = true;
	// TODOPING do things here
	destDistanceMM = VL53L0X_get_dist_mm();
	return destIsNotReached;
}

/**
 * @brief   Updates the speeds of the motors based on distance and angle
*/
void motControllerUpdate(void){
	//TODOPING first speed differential based on angle
	int16_t motSpeedDiff = 0;
	int16_t sumLastNAngles = 0;

	if(destAngle > MOT_MAX_ANGLE_TO_CORRECT)
		destAngle = MOT_MAX_ANGLE_TO_CORRECT;
	else if(destAngle < (- MOT_MAX_ANGLE_TO_CORRECT) )
		destAngle = (- MOT_MAX_ANGLE_TO_CORRECT);
	// shift angles to add newest obeserved one (do this here because it is at regular intervals and do sum
	for(uint8_t i = MOT_KI_N_ANGLES - 1; i>0;i--){ //1 offset because it's an array
		lastNAngles[i] = lastNAngles[i-1]; // shift all angles (discard oldest one)
		sumLastNAngles+=lastNAngles[i];
	}
	lastNAngles[0] = destAngle;
	sumLastNAngles+=lastNAngles[0];

	motSpeedDiff = MOT_KP_DIFF*destAngle + MOT_KI_DIFF*(sumLastNAngles);

	if(motSpeedDiff > MOT_MAX_DIFF_SPS_FOR_CORRECTION)
		motSpeedDiff = MOT_MAX_DIFF_SPS_FOR_CORRECTION;
	else if( motSpeedDiff < (- MOT_MAX_DIFF_SPS_FOR_CORRECTION))
		motSpeedDiff = (- MOT_MAX_DIFF_SPS_FOR_CORRECTION);

	// TODOPING then update speed based on distance
	int robSpeed = 0;
	if(STOP_DISTANCE_VALUE_MM < destDistanceMM && destDistanceMM < MAX_DISTANCE_VALUE_MM)
		robSpeed = ( MOT_MAX_NEEDED_SPS * (destDistanceMM-STOP_DISTANCE_VALUE_MM) )/MAX_DISTANCE_VALUE_MM;
	else if(destDistanceMM == MAX_DISTANCE_VALUE_MM)
		robSpeed = MOT_MAX_NEEDED_SPS;

	// TODOPING then actually update motor speeds
	rightMotSpeed = robSpeed + motSpeedDiff;
	leftMotSpeed = robSpeed - motSpeedDiff;
	right_motor_set_speed(rightMotSpeed);
	left_motor_set_speed(leftMotSpeed);
}

// TODOPING either delete this or change it to function for terminating the controller from outside
///**
// * @brief   function to be called when the destination is reached according to prox sensor
//*/
//void destinationReached(void){
//	// set termination flag to true in motCtrlThread
//	chThdTerminate(motCtrlThread);
//	chThdWait(motCtrlThread); // wait until thread has effectively stopped
//	destReachedFctToCall();
//}

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

	VL53L0X_start();

	destAngle = 0;
	destDistanceMM = 0;

	destReachedFctToCall = destReachedCallback;

	//start of controller thread here
	motCtrlThread = 	chThdCreateStatic(waMotControllerThd, sizeof(waMotControllerThd), NORMALPRIO, MotControllerThd, NULL);

	return &dirAngleCb;
}

/*===========================================================================*/
/* Functions for testing              */
/*===========================================================================*/
// TESTPING
//void travCtrl_testAll(void){
//	travCtrl_dirAngleCb_t updateAngleCB = travCtrl_init();
//	int16_t testNewAngle = 50;
//
//	updateAngleCB(testNewAngle);
//
//	chThdSleepMilliseconds(1000);
//
//}
