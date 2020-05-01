/*
 * travelController.h
 *
 *  Created on: April 1, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * and stops when an obstacle/the objective is reached (detection with proximity sensor).
 */

#ifndef TRAVELCONTROLLER_H_
#define TRAVELCONTROLLER_H_

/*
 * @brief type for callback function when obstacle is reached
 * @note is used to declare a pointer to a function that will be called when an obstacle has been reached.
 * 			Use this as example for how to define the function you provide on initialisation
 */
typedef void (*travCtrl_obstacleReached)(void);

/**
 * @brief   To start the whole controller, for later moving towards destination.
 * @parameter[in] obstacleReachedCallBackPointer callback for when destination is reached
*/
void travCtrl_init(travCtrl_obstacleReached obstacleReachedCallBackPointer);

/**
 * @brief   Will set the robot moving towards provided angle, or if already moving
 * 				will just update the direction of movement
 * @parameter[in] directionAngle 	direction to go to, between -179° and 180°	//NICOLAJ final check is this -179° true ?
*/
void travelCtrl_goToAngle(int16_t directionAngle);

/**
 * @brief   stops movements until new instructions are given
*/
void travCtrl_stopMoving(void);

#endif /* TRAVELCONTROLLER_H_ */
