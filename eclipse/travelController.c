/*
 * travelController.c
 *
 *  Created on: May 7, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * 		and stops when an obstacle/the objective is reached (detection with proximity sensor).
 *
 * Functions prefix for public functions in this file: travCtrl_
 */

#include <ch.h> 							//for chibios threads functionality

#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>		//Time of flight sensor library
#include <sensors/proximity.h>			//infrared sensors

#include <travelController.h>
#include "msgbus/messagebus.h"



/*===========================================================================*/
/* Constants definition for this file						               */
/*===========================================================================*/

/* @note MOT_MAX_NEEDED_SPS
 * motors.h library functions need a steps per second (sps) max speed of
 * 1100steps/s (MOTOR_SPEED_LIMIT). Thus, do not set MOT_MAX_NEEDED_SPS above
 * MOTOR_SPEED_LIMIT (=1100) - MOT_MAX_DIFF_SPS_FOR_CORRECTION - MOT_MIN_SPEED_SPS
 * otherwise the robot might not be able to turn normally, as speeds might truncate. */
#define MOT_MAX_NEEDED_SPS 					500

/* @note MOT_MAX_DIFF_SPS_FOR_CORRECTION
 * it is the speed in steps/s over which the controller will not go faster.  It must be less than
 * MOTOR_SPEED_LIMIT (=1100) - MOT_MAX_NEEDED_SPS - MOT_MIN_SPEED_SPS */
#define MOT_MAX_DIFF_SPS_FOR_CORRECTION 		222

/* @note MOT_MIN_SPEED_SPS
 * we saw that under MOT_MIN_SPEED_SPS steps/s the motors were vibrating, so this is the minimum speed in steps/s we will set */
#define MOT_MIN_SPEED_SPS 					150

#define MAX_DISTANCE_VALUE_MM 				350 			//how far, in mm, from an obstacle should the robot start to slow down
#define STOP_DISTANCE_VALUE_MM				35			//how far, in mm, from an obstacle should the robot stop moving
#define INIT_DISTANCE_VALUE_MM				80

#define MOT_MAX_ANGLE_TO_CORRECT 			40			//in degrees, if angle is bigger robot only turns at a constant rotating speed

#define MOT_CONTROLLER_PERIOD 				10 			//in ms, will be the interval at which controller thread will re-adjust motor speeds
#define MOT_CONTROLLER_WORKING_AREA_SIZE 	1024 		//1024 because it was found to be enough: less results in seg faults

/* @note EMA_WEIGHT
 * We use this to calculate exponential moving averages (ema) of some values, in order to reduce
 * impact of fluctuations that are too quick to represent real changes. We calculate an ema like this :
 * If faster response time overall is needed, reduce EMA_WEIGHT.
 */
#define EMA_WEIGHT 							0.9
#define EMA_WEIGHT_TOF 						0.8

/* @note DISCARD_FIRST_N_TOF_MEASURES
 * The time of flight sensor returns 0 values for approx 30 cycles, so we set 50 to have a margin and discard those first cycles
 */
#define DISCARD_FIRST_N_TOF_MEASURES 		50

#define IR_STOP_VALUE						300	//IR threshold for source detection
#define IR_LEFT 								6	//EPUCK IR-seonsor IR7
#define IR_RIGHT								1	//EPUCK IR-seonsor IR2
#define IR_FRONT_LEFT 						7	//EPUCK IR-seonsor IR8
#define IR_FRONT_RIGHT 						0	//EPUCK IR-seonsor IR1



/*===========================================================================*/
/* Static variables definitions 		 			                            */
/*===========================================================================*/

static int16_t destAngle = 0; 			//from -180° to +180°

static uint16_t emaPastDistances = 0;	//Exponential moving average (must be ema otherwise it is too jumpy)

static bool robShouldMove = false;		//the motor controller will only update speeds when this is true

/* @note obstacleReachedCallBack
 * pointer to function that is provided upon initialization, which we call when we arrive at an obstacle */
static travCtrl_obstacleReached obstacleReachedCallBack;



/*===========================================================================*/
/* Variables and functions for the use of the IR sensors						*/
/*===========================================================================*/
extern messagebus_t bus;
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



/*===========================================================================*/
/* Private functions definitions, no comments as they are already above function code*/
/*===========================================================================*/

/**
 * @brief   Updates the measured distance to first object in sight
 * @return  true if obstacle is reached, false otherwise
*/
bool updateIsObstacleReached(void);

/**
 * @brief   Updates the speeds of the motors based on distance and angle to obstace
*/
void motControllerUpdateSpeeds(void);

/**
 * @brief   calculates the speed differential (rotational speed) of the motors based on direction angle
 * @note 	The angle is positive to the right. This function calculates a positive differential for a positive angle.
 * 				To use this differential speed, when angle is positive and the robot needs to turn right, you will have
 * 				to add this differential speed to the left motor, so it will go faster, and substract this differential
 * 				speed to the right motor, so it will go slower. This will results in a turn in the direction wanted.
 *
 * @return	The calculated value for the new speed differential, in steps per second,
 * 			between -MOT_MAX_DIFF_SPS_FOR_CORRECTION and MOT_MAX_DIFF_SPS_FOR_CORRECTION
*/
int16_t motControllerCalculatetRotationSpeed(void);

/**
 * @brief   calculates the common speed for the motors motors based on distance
 * @return 	The calculated speed for the motors in steps per second, between 0 and MOT_MAX_NEEDED_SPS
*/
uint16_t motControllerCalculateForwardSpeed(void);



/*===========================================================================*/
/* Threads used in travelController                  							*/
/*===========================================================================*/

/* Working area for the motor controller thread */
static THD_WORKING_AREA(waMotControllerThd, MOT_CONTROLLER_WORKING_AREA_SIZE);
/* Motor controller thread. */
static THD_FUNCTION(MotControllerThd, arg)
{
	(void)arg; 									// silence warning about unused argument
	while (true) {
		systime_t time = chVTGetSystemTime();	//time to restart it MOT_CONTROLLER_PERIOD milliseconds later

		if(robShouldMove){

			//when an obstacle is reached we stop moving and use the callback provided on initialization
			if(updateIsObstacleReached() == true){
				travCtrl_stopMoving();
				obstacleReachedCallBack();
			}
			else{
				motControllerUpdateSpeeds();		//when the obstacle isn't reached, we run the controller
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(MOT_CONTROLLER_PERIOD));	//the thread runs every MOT_CONTROLLER_PERIOD milliseconds
	}
}


/*===========================================================================*/
/* Public functions for setting/getting internal parameters					  */
/* Descriptions  are only in the header file (to have a single source of truth) */
/*===========================================================================*/

void travCtrl_init(travCtrl_obstacleReached obstacleReachedCallBackPointer)
{
	//Start chibiOS modules: motor & time of flight (TOF) sensor
	motors_init();
	VL53L0X_start();

	//Start and calibrate the infrared proximity sensors
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	calibrate_ir();

	// Update file level function pointer to callback provided for when destination is reached
	obstacleReachedCallBack = obstacleReachedCallBackPointer;

	/* Start of the controller thread here. We never stop the thread, only skip some code inside
	 * it when not needed. Therefore, we do not remember the pointer to the thread.
	 * The priority is set to NORMALPRIO even though it is a critical task, because
	 * other running threads must also run for the controller to be useful, so
	 * all threads should have the same priority as  MotControllerThd*/
	chThdCreateStatic(waMotControllerThd, sizeof(waMotControllerThd), NORMALPRIO, MotControllerThd, NULL);

	return;
}

void travelCtrl_goToAngle(int16_t directionAngle)
{
	destAngle = directionAngle;
	robShouldMove=true;
}

void travCtrl_stopMoving()
{
	robShouldMove=false;			// This file variable makes the thread skip controller functions if false

	right_motor_set_speed(0);	//We need to actually stop the motors, or they will keep the last values set.
	left_motor_set_speed(0);

}

void travCtrl_moveBackwards(void)
{
	right_motor_set_speed(-MOT_MAX_NEEDED_SPS);
	left_motor_set_speed(-MOT_MAX_NEEDED_SPS);
}



/*===========================================================================*/
/* Private functions	 code												   */
/*===========================================================================*/

bool updateIsObstacleReached(void)
{
	static uint8_t discardStartMeasurements 	= 0;

	static int emaPastValueLeft 				= 0;
	static int emaPastValueRight 			= 0;
	static int emaPastValueFrontLeft			= 0;
	static int emaPastValueFrontRight		= 0;

	bool isDestinationReached = false;

	// update filtered emaPastDistances value with newest value
	emaPastDistances = (int16_t) (EMA_WEIGHT_TOF*emaPastDistances+(1-EMA_WEIGHT_TOF)*VL53L0X_get_dist_mm() );

	emaPastValueLeft=get_calibrated_prox(IR_LEFT);
	emaPastValueRight=get_calibrated_prox(IR_RIGHT);
	emaPastValueFrontLeft=get_calibrated_prox(IR_FRONT_LEFT);
	emaPastValueFrontRight=get_calibrated_prox(IR_FRONT_RIGHT);

	//Set isDestinationReached to true when an object is detected from the FOT-seonsor or the IR-sensor
	if(emaPastDistances<=STOP_DISTANCE_VALUE_MM || emaPastValueLeft>IR_STOP_VALUE || emaPastValueRight>IR_STOP_VALUE ||
												emaPastValueFrontLeft>IR_STOP_VALUE || emaPastValueFrontRight>IR_STOP_VALUE){

		emaPastDistances = INIT_DISTANCE_VALUE_MM;		//Reinitialize TOF-distance for further use
		isDestinationReached = true;
	}

	//For approx. the first DISCARD_FIRST_N_TOF_MEASURES times VL53L0X_get_dist_mm() is called it returns 0.
	//We check if we are in the first DISCARD_FIRST_N_TOF_MEASURES cycles and discard these cycles
	if(discardStartMeasurements<DISCARD_FIRST_N_TOF_MEASURES){
		isDestinationReached = false;
		discardStartMeasurements++;
		emaPastDistances = INIT_DISTANCE_VALUE_MM;
	}

	return isDestinationReached;
}


void motControllerUpdateSpeeds(void)
{
	static int16_t ema_rightMotSpeed 	= 0;
	static int16_t ema_leftMotSpeed 		= 0;

	int16_t rightMotSpeed 				= 0;						// In steps per second
	int16_t leftMotSpeed 				= 0;						// In steps per second

	// We first obtain the rotational (differential) speed (based on angle).
	int16_t motSpeedDiff = motControllerCalculatetRotationSpeed();

	//Then we obtain the forward speed (based on distance)
	uint16_t robForwardSpeed 			= 0;

	if(-MOT_MAX_ANGLE_TO_CORRECT<destAngle && destAngle<MOT_MAX_ANGLE_TO_CORRECT){
		robForwardSpeed = motControllerCalculateForwardSpeed();
	}

	//The motor speeds (before filtering) are calculated with the forward speed and differential speed.
	rightMotSpeed = (int16_t) robForwardSpeed - (int16_t) motSpeedDiff;
	leftMotSpeed = (int16_t) robForwardSpeed + (int16_t) motSpeedDiff;

	//We use exponential moving average values because speeds must not be changed to fast or motors make grinding noises
	ema_rightMotSpeed = (int16_t) (EMA_WEIGHT*ema_rightMotSpeed+(1-EMA_WEIGHT)*rightMotSpeed);
	ema_leftMotSpeed = (int16_t) (EMA_WEIGHT*ema_leftMotSpeed+(1-EMA_WEIGHT)*leftMotSpeed);

	/*We saw that when speeds were below MOT_MIN_SPEED_SPS steps per second,
	 * the motors were vibrating, so here we offset xxxMotSpeed values,
	 * first the right speed, then the left. We do this after the exponential moving average
	 * but it is not a problem (it could be if values oscillated around 0) since we only
	 * have values near 0 when we reach the object. Thus, the robot stops and there is
	 * never oscillations near a speed of 0 */
	if(ema_rightMotSpeed>0){
		rightMotSpeed = ema_rightMotSpeed+MOT_MIN_SPEED_SPS;
	}
	else if(ema_rightMotSpeed<0){
		rightMotSpeed = ema_rightMotSpeed-MOT_MIN_SPEED_SPS;
	}
	else{
		rightMotSpeed=0;		//we set this explicitly but this case happens very rarely, just before stopping
	}

	if(ema_leftMotSpeed>0){
		leftMotSpeed = ema_leftMotSpeed+MOT_MIN_SPEED_SPS;
	}
	else if(ema_leftMotSpeed<0){
		leftMotSpeed = ema_leftMotSpeed-MOT_MIN_SPEED_SPS;
	}
	else{
		leftMotSpeed=0;		//we set this explicitly but this case happens very rarely, just before stopping
	}

	//Set the motor speeds
	right_motor_set_speed(rightMotSpeed);
	left_motor_set_speed(leftMotSpeed);
}

int16_t motControllerCalculatetRotationSpeed(void)
{
	int16_t motSpeedDiff 	= 0;				// to store the rotational (differential) motor speed in steps/s

	int16_t tempDestAngle 	= destAngle;		//we do not want to modify destAngle, so just copy it

	/* Our controller only changes values for angles up to MOT_MAX_ANGLE_TO_CORRECT,
	 * otherwise it applies the max correction*/
	if(tempDestAngle > MOT_MAX_ANGLE_TO_CORRECT){
		tempDestAngle = MOT_MAX_ANGLE_TO_CORRECT;
	}
	else if(tempDestAngle < (- (int16_t) MOT_MAX_ANGLE_TO_CORRECT) ){
		tempDestAngle = (- (int16_t) MOT_MAX_ANGLE_TO_CORRECT);
	}

	/* Here we calculate the differential, in steps per second, which will always
	 * 	between -MOT_MAX_DIFF_SPS_FOR_CORRECTION and MOT_MAX_DIFF_SPS_FOR_CORRECTION.
	 * 	Rounding to integer value is wanted, and not a problem as speed is in integer steps/s */
	motSpeedDiff = MOT_MAX_DIFF_SPS_FOR_CORRECTION * tempDestAngle / MOT_MAX_ANGLE_TO_CORRECT;

	return motSpeedDiff;
}


uint16_t motControllerCalculateForwardSpeed(void)
{
	uint16_t robSpeed = 0;					//to store the forward (common speed for both motors) in steps/s

	/* When the distance is between min and max values, then calculate robSpeed with proportional
	 * controller. Otherwise if further than max distance, we just set max speed, or else leave
	 * 0 speed as it means that an object is reached.
	 * Rounding to integer value is wanted, and not a problem as speed is in integer steps/s */
	if(STOP_DISTANCE_VALUE_MM <= emaPastDistances && emaPastDistances <= MAX_DISTANCE_VALUE_MM){

		robSpeed = ( MOT_MAX_NEEDED_SPS * (emaPastDistances-STOP_DISTANCE_VALUE_MM) )/(MAX_DISTANCE_VALUE_MM-STOP_DISTANCE_VALUE_MM);

	}
	else if(emaPastDistances > MAX_DISTANCE_VALUE_MM){

		robSpeed = MOT_MAX_NEEDED_SPS;

	}
	//if none of above two cases are true robSpeed is kept at 0  because it means an object is closer than MAX_DISTANCE_VALUE_MM

	return robSpeed;
}


