/*
 * travelController.c
 *
 *  Created on: Apr 2, 2020
 *      Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * and stops when an obstacle/the objective is reached (detection with proximity sensor).
 * Functions prefix for public functions in this file: travCtrl_
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ch.h> //for threads

#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <travelController.h>
#include <comms.h>

// From motors.h library functions need steps/s max speed 1100steps/s (MOTOR_SPEED_LIMIT) but for us might need less
// @warning do not set speed above MOTOR_SPEED_LIMIT (=1100) - MOT_MAX_DIFF_SPS_FOR_CORRECTION otherwise it couldn't turn !
#define MOT_MAX_NEEDED_SPS 300

#define MAX_DISTANCE_VALUE_MM 500 //how far in mm should robot start to slow
#define STOP_DISTANCE_VALUE_MM 30 //how far in mm should robot stop
#define STOP_DISTANCE_AVERAGE_N 3 //to filter too high variations

#define MOT_MAX_ANGLE_TO_CORRECT 60	// this will be the max angle in ° that the correction will still change
#define MOT_MAX_DIFF_SPS_FOR_CORRECTION 300 // must be less than MOTOR_SPEED_LIMIT (=1100) - MOT_MAX_NEEDED_SPS
//#define MOT_CORRECTION_EXPONENT 2.5 //can range from 1 (no less than linear) to technically anything, and with decimals
#define MOT_KP_DIFF 1	//needs >=0 value
#define MOT_KI_DIFF 0.1 //needs >=0 value
#define MOT_KI_N_ANGLES 5
#define MOT_KP_FWD 0.5 //forward speed KP needs >=0 value
#define MOT_KI_FWD 1 //forward speed KI needs >=0 value


#define MOT_CONTROLLER_PERIOD 10 //in ms, will be the interval at which controller thread will re-adjust control
#define MOT_CONTROLLER_WORKING_AREA_SIZE 1024 //128 because it should be enough !

#define IR_FRONT_RIGHT 0 //IR1 so sensor number 0
#define IR_FRONT_LEFT 7 //IR8 so sensor number 7
#define IR_CALIB_0_DISTANCE -3700 //from http://www.e-puck.org/index.php?option=com_content&view=article&id=22&Itemid=13
#define IR_CALIB_MAX_RANGE -750 //same source, actually only somewhat linear bewteen -3700 and -1000

int16_t destAngle = 0; //from -179 to +180
int16_t lastNAngles[MOT_KI_N_ANGLES] = {0}; //set all to 0
uint16_t destDistanceMM = 0;
uint16_t lastNdestDistanceMM[STOP_DISTANCE_AVERAGE_N] = {0};
int rightMotSpeed = 0; //from -126 to +126, it is an int as in motors.h
int leftMotSpeed = 0; //from -126 to +126, it is an int as in motors.h
thread_t *motCtrlThread; // pointer to motor controller thread if needed to stop it TODOPING maybe remove if not necessary anymore

bool robShouldMove = false;
//TESTPING
bool degubPrintf = true;

travCtrl_destReached destReachedFctToCall;

/*===========================================================================*/
/* Internal functions definitions             */
/*===========================================================================*/
void dirAngleCb(int16_t newDestAngle);
bool proxDistanceUpdate(void);
void motControllerUpdate(void);
int16_t motControllerCalculatetSpeedDiff(void);
int motControllerCalculateSpeed(void);

/*===========================================================================*/
/* Private functions              */
/*===========================================================================*/

/* Working area for the motor controller. */
static THD_WORKING_AREA(waMotControllerThd, MOT_CONTROLLER_WORKING_AREA_SIZE);
/* Motor controller thread. */
static THD_FUNCTION(MotControllerThd, arg) {
	(void)arg; // silence warning about unused argument
    systime_t time;
	while (true) {
		time = chVTGetSystemTime();
		if(robShouldMove){
			bool motCtrShldContinue = proxDistanceUpdate();
			if(motCtrShldContinue==false){
				robShouldMove=false;
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				destReachedFctToCall();
			}
			else{
				motControllerUpdate();
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(MOT_CONTROLLER_PERIOD));
	}
}

/**
 * @brief   Updates the measured distance to object
 * @return  false if destination is not reached 0, true otherwise
*/
bool proxDistanceUpdate(void){
	bool destIsNotReached = true;

	destDistanceMM = VL53L0X_get_dist_mm();

	//TODOPING first times gestDist is calledd returns 0 apparantly so filter those first...
	static uint8_t testFirstGistances = 0;
	testFirstGistances++;
	if(testFirstGistances>50 && destDistanceMM <= STOP_DISTANCE_VALUE_MM ){
		destIsNotReached = false;
		testFirstGistances = 100; //TODOPING check how to remove this magic number
	}

	return destIsNotReached;
}


/**
 * @brief   Updates the speed differential of the motors based on angle
 * @return	The calculated value for the new speed differential, in steps per second,
 * 			between -MOT_MAX_DIFF_SPS_FOR_CORRECTION and MOT_MAX_DIFF_SPS_FOR_CORRECTION
*/
int16_t motControllerCalculatetSpeedDiff(void){
	int16_t motSpeedDiff = 0;
	int16_t avgLastNAngles = 0;

	if(destAngle > MOT_MAX_ANGLE_TO_CORRECT)
		destAngle = MOT_MAX_ANGLE_TO_CORRECT;
	else if(destAngle < (- MOT_MAX_ANGLE_TO_CORRECT) )
		destAngle = (- MOT_MAX_ANGLE_TO_CORRECT);
	// shift angles to add newest obeserved one (do this here because it is at regular intervals and do sum), and do average
	for(uint8_t i = MOT_KI_N_ANGLES - 1; i>=1;i--){ //1 offset because it's an array
		lastNAngles[i] = lastNAngles[i-1]; // shift all angles (discard oldest one)
		avgLastNAngles+=lastNAngles[i];
	}
	lastNAngles[0] = destAngle;
	avgLastNAngles+=lastNAngles[0];
	avgLastNAngles = avgLastNAngles/MOT_KI_N_ANGLES;

//	motSpeedDiff = MOT_KP_DIFF * MOT_MAX_DIFF_SPS_FOR_CORRECTION * destAngle / MOT_MAX_ANGLE_TO_CORRECT
//					+ MOT_KI_DIFF* MOT_MAX_DIFF_SPS_FOR_CORRECTION * avgLastNAngles / MOT_MAX_ANGLE_TO_CORRECT;
	motSpeedDiff = MOT_KP_DIFF * MOT_MAX_DIFF_SPS_FOR_CORRECTION * destAngle / MOT_MAX_ANGLE_TO_CORRECT;

	if(motSpeedDiff > MOT_MAX_DIFF_SPS_FOR_CORRECTION)
		motSpeedDiff = MOT_MAX_DIFF_SPS_FOR_CORRECTION;
	else if( motSpeedDiff < (- MOT_MAX_DIFF_SPS_FOR_CORRECTION))
		motSpeedDiff = (- MOT_MAX_DIFF_SPS_FOR_CORRECTION);

	return motSpeedDiff;
}

/**
 * @brief   Updates the speed for both motors based on distance
 * @return 	The calculated speed for the motors in steps per second, between 0 and MOT_MAX_NEEDED_SPS
*/
int motControllerCalculateSpeed(void){
	int robSpeed = 0; //TODOPING why int and not uint16_t here ?

	// first : filter distances using KP KI controller
	uint16_t sumLastNdestDistanceMM = 0;
	// ---- shift last N distances in array
	for(uint8_t i = STOP_DISTANCE_AVERAGE_N - 1; i>=1;i--){ //1 offset because it's an array
		lastNdestDistanceMM[i] = lastNdestDistanceMM[i-1]; // shift all angles (discard oldest one)
		sumLastNdestDistanceMM+=lastNdestDistanceMM[i];
		}
	lastNdestDistanceMM[0] = destDistanceMM;
	sumLastNdestDistanceMM+=lastNdestDistanceMM[0];

	// -- Calculate filtered destDistanceMM value
	destDistanceMM = MOT_KP_FWD*destDistanceMM + MOT_KI_FWD*sumLastNdestDistanceMM;

	// if in controller bounds, then calculate robSpeed with parameters
	if(STOP_DISTANCE_VALUE_MM <= destDistanceMM && destDistanceMM <= MAX_DISTANCE_VALUE_MM){
		robSpeed = ( MOT_MAX_NEEDED_SPS * (destDistanceMM-STOP_DISTANCE_VALUE_MM) )/(MAX_DISTANCE_VALUE_MM-STOP_DISTANCE_VALUE_MM);
		if(robSpeed<150)	//TESTPING weird here...
			robSpeed = 150;
	}
	else if(destDistanceMM > MAX_DISTANCE_VALUE_MM)
		robSpeed = MOT_MAX_NEEDED_SPS;

	return robSpeed;
}

/**
 * @brief   Updates the speeds of the motors based on distance and angle
*/
void motControllerUpdate(void){
	//First : control speed differential based on angle
	int16_t motSpeedDiff = motControllerCalculatetSpeedDiff();

	// Then update speed based on distance
	uint16_t robSpeed = motControllerCalculateSpeed();

	// Then actually update motor speeds
	rightMotSpeed = robSpeed - motSpeedDiff;
	leftMotSpeed = robSpeed + motSpeedDiff;
	//TESTPING not outputting values to motors because for now need to check if it works ok
	//if(degubPrintf == true)
	//chprintf(UART_PORT_STREAM, "New rob speeds are : Left = %d, Right=%d, distanceMM = %d\n\r", leftMotSpeed,rightMotSpeed,destDistanceMM);

	right_motor_set_speed(rightMotSpeed);
	left_motor_set_speed(leftMotSpeed);
}

/*===========================================================================*/
/* Public functions for setting/getting internal parameters             */
/*===========================================================================*/

void travCtrl_init(travCtrl_destReached destReachedCallback){
	// start chibiOS modules: motor & TOF sensor
	motors_init();
	VL53L0X_start();

	//update file level function pointer to callback provided for when destination is reached
	destReachedFctToCall = destReachedCallback;

	//start of controller thread here
	motCtrlThread = 	chThdCreateStatic(waMotControllerThd, sizeof(waMotControllerThd), NORMALPRIO, MotControllerThd, NULL);
}

void travCtrl_stopMoving(){
	robShouldMove=false;
}

void travelCtrl_goToAngle(int16_t directionAngle){
	destAngle = directionAngle;
}
