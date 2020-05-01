/*
 * travelController.c
 *
 *  Created on: Apr 1, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * and stops when an obstacle/the objective is reached (detection with proximity sensor).
 *
 * Functions prefix for public functions in this file: travCtrl_
 */

#include <ch.h> //for chibios threads functionality

#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>	//Time of flight sensor library

#include <travelController.h>

/* @note MOT_MAX_NEEDED_SPS
 * motors.h library functions need a steps per second (sps) max speed of
 * 1100steps/s (MOTOR_SPEED_LIMIT). Thus, do not set MOT_MAX_NEEDED_SPS above
 * MOTOR_SPEED_LIMIT (=1100) - MOT_MAX_DIFF_SPS_FOR_CORRECTION - MOT_MIN_SPEED_SPS
 * otherwise the robot might not be able to turn normally, as speeds might truncate. */
#define MOT_MAX_NEEDED_SPS 333	//Experimental value that seemed to work well

#define MAX_DISTANCE_VALUE_MM 500 //how far, in mm, from an obstacle should the robot start to slow down
#define STOP_DISTANCE_VALUE_MM 30 //how far, in mm, from an obstacle should the robot stop moving

#define MOT_MAX_ANGLE_TO_CORRECT 40	// angle in ° over which the motor will not move forward, only turn, and at a constant rotating speed
/* @note MOT_MAX_DIFF_SPS_FOR_CORRECTION
 * it is the speed in steps/s over which the controller will not go faster.  It must be less than
 * MOTOR_SPEED_LIMIT (=1100) - MOT_MAX_NEEDED_SPS - MOT_MIN_SPEED_SPS */
#define MOT_MAX_DIFF_SPS_FOR_CORRECTION 222
/* @note MOT_MIN_SPEED_SPS
 * we saw that under under 100steps/s the motors were vibrating, so this is the minimum speed in steps/s we will set */
#define MOT_MIN_SPEED_SPS 100

#define MOT_CONTROLLER_PERIOD 10 //in ms, will be the interval at which controller thread will re-adjust motor speeds
#define MOT_CONTROLLER_WORKING_AREA_SIZE 1024 //1024 because it was found to be enoug: less results in seg faults

/*@note EMA_WEIGHT
 * We use this to calculate exponential moving averages of some values, in order to reduce
 * impact of fluctuations that are too quick to represent real changes. We calculate an ema like this :
 * emaVariable = (EMA_WEIGHT*emaVariable+(1-EMA_WEIGHT)*variable )
 * Using EMA_WEIGHT and 1-EMA_WEIGHT insures we stay withing the bounds of what values the unaveraged variable can take.
 * If faster response time overall is needed, reduce this constant.
 */
#define EMA_WEIGHT 0.9


static int16_t destAngle = 0; //from -179 to +180
//TODOPING static int16_t ema_destAngle = 0; //to store the exponention moving average of previous destAngle values
static uint16_t emaPastDistances = 0; //We use an exponential moving average of past distance values because it was too jumpy otherwise
static thread_t *motCtrlThread; // pointer to motor controller thread if needed to stop it TODOPING maybe remove if not necessary anymore

static bool robShouldMove = false;

travCtrl_destReached destReachedFctToCall;

/*===========================================================================*/
/* Internal functions definitions             */
/*===========================================================================*/
void dirAngleCb(int16_t newDestAngle);
bool proxDistanceUpdate(void);
void motControllerUpdate(void);
int16_t motControllerCalculatetSpeedDiff(void);
uint16_t motControllerCalculateSpeed(void);

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
	//TODOPING first times gestDist is calledd returns 0 apparantly so filter those first...
	static uint8_t testFirstGistances = 0;

	bool destIsNotReached = true;

	// update filtered emaPastDistances value with newest value
	emaPastDistances = (int16_t) (EMA_WEIGHT*emaPastDistances+(1-EMA_WEIGHT)*VL53L0X_get_dist_mm() );

	testFirstGistances++;
	if(testFirstGistances>50 && emaPastDistances <= STOP_DISTANCE_VALUE_MM ){	//TODOPING constant here !
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
	//int16_t avgLastNAngles = 0;

	int16_t tempDestAngle = destAngle;

	//our controller only changes values for angles up to MOT_MAX_ANGLE_TO_CORRECT, otherwise it is the max correction applied
	//If angle is larger than MOT_MAX_ANGLE_TO_CORRECT, then we correct the same as angle MOT_MAX_ANGLE_TO_CORRECT
	if(tempDestAngle > MOT_MAX_ANGLE_TO_CORRECT){
		tempDestAngle = MOT_MAX_ANGLE_TO_CORRECT;
	}
	else if(tempDestAngle < (- (int16_t) MOT_MAX_ANGLE_TO_CORRECT) ){
		tempDestAngle = (- (int16_t) MOT_MAX_ANGLE_TO_CORRECT);
	}

	motSpeedDiff = MOT_MAX_DIFF_SPS_FOR_CORRECTION * tempDestAngle / MOT_MAX_ANGLE_TO_CORRECT;

	return motSpeedDiff;
}

/**
 * @brief   calculates the common speed for the motors motors based on distance
 * @return 	The calculated speed for the motors in steps per second, between 0 and MOT_MAX_NEEDED_SPS
*/
uint16_t motControllerCalculateSpeed(void){
	uint16_t robSpeed = 0;

	// when distance is between min and max values, then calculate robSpeed with proportionnal controller
	// otherwise if further than max distance, we just set max speed, or else leave 0 speed as destination is reached.
	if(STOP_DISTANCE_VALUE_MM <= emaPastDistances && emaPastDistances <= MAX_DISTANCE_VALUE_MM){
		robSpeed = ( MOT_MAX_NEEDED_SPS * (emaPastDistances-STOP_DISTANCE_VALUE_MM) )/(MAX_DISTANCE_VALUE_MM-STOP_DISTANCE_VALUE_MM);
		if(robSpeed<150)	//TESTPING weird here...
			robSpeed = 150;
	}
	else if(emaPastDistances > MAX_DISTANCE_VALUE_MM){
		robSpeed = MOT_MAX_NEEDED_SPS;
	}

	return robSpeed;
}

/**
 * @brief   Updates the speeds of the motors based on distance and angle
*/
void motControllerUpdate(void){
	int16_t rightMotSpeed = 0; //from -126 to +126, it is an int as in motors.h
	int16_t leftMotSpeed = 0; //from -126 to +126, it is an int as in motors.h
	static int16_t ema_rightMotSpeed = 0;	//To store the exponential moving average of motor speeds so as not to break motors changing too rapidly
	static int16_t ema_leftMotSpeed = 0;
	int16_t finalRightMotSpeed = 0;	//Used for the final offset speed
	int16_t finalLeftMotSpeed = 0;

	//First : control speed differential based on angle
	int16_t motSpeedDiff = motControllerCalculatetSpeedDiff();
	uint16_t robSpeed = 0;

	// Then update speed based on distance but only if source is front of robot
	if(-60<destAngle && destAngle<60){ //TODOPING constants !!!
		robSpeed = motControllerCalculateSpeed();
	}


	// Then actually update motor speeds
	rightMotSpeed = (int16_t) robSpeed - (int16_t) motSpeedDiff;
	leftMotSpeed = (int16_t) robSpeed + (int16_t) motSpeedDiff;


	//TESTPING not outputting values to motors because for now need to check if it works ok
	//if(degubPrintf == true)


	ema_rightMotSpeed = (int16_t) (EMA_WEIGHT*ema_rightMotSpeed+(1-EMA_WEIGHT)*rightMotSpeed);
	ema_leftMotSpeed = (int16_t) (EMA_WEIGHT*ema_leftMotSpeed+(1-EMA_WEIGHT)*leftMotSpeed);

	//We saw that when speeds were below MOT_MIN_SPEED_SPS steps per second, the motors were vibrating, so here we offset speeds
	//Right speed offset
	if(ema_rightMotSpeed>0){
		finalRightMotSpeed = ema_rightMotSpeed+MOT_MIN_SPEED_SPS;
	}
	else if(ema_rightMotSpeed<0){
		finalRightMotSpeed = ema_rightMotSpeed-MOT_MIN_SPEED_SPS;
	}
	else{
		finalRightMotSpeed=0;
	}
	//Now left speed offset
	if(ema_leftMotSpeed>0){
		finalLeftMotSpeed = ema_leftMotSpeed+MOT_MIN_SPEED_SPS;
	}
	else if(ema_leftMotSpeed<0){
		finalLeftMotSpeed = ema_leftMotSpeed-MOT_MIN_SPEED_SPS;
	}
	else{
		finalLeftMotSpeed=0;
	}
//	comms_printf(UART_PORT_STREAM, "destAngle = %d    leftSpeed = %d,   	rightSpeed=%d,	   distanceMM = %d, 	  motSpeedDiff=%d, \n\r", destAngle, finalLeftMotSpeed,finalRightMotSpeed,destDistanceMM, motSpeedDiff);

	right_motor_set_speed(finalRightMotSpeed);
	left_motor_set_speed(finalLeftMotSpeed);
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
	right_motor_set_speed(0);
	left_motor_set_speed(0);

}

void travelCtrl_goToAngle(int16_t directionAngle){
	destAngle = directionAngle;
	robShouldMove=true;
}
