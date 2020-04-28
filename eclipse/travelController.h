/*
 * travelController.h
 *
 *  Created on: April 2, 2020
 *      Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * and stops when an obstacle/the objective is reached (detection with proximity sensor).
 */

#ifndef TRAVELCONTROLLER_H_
#define TRAVELCONTROLLER_H_

#include <stdio.h>

/* @brief type for callback function in order to give new direction angle
* @note this is a pointer type, to a travCtrl_dirAngleCb_t type function
*   can be called from the pointer by using standard ()
* @parameter [in] give an integer value between -179 to +180°
*/
typedef void (*travCtrl_dirAngleCb_t)(int16_t);

// @brief type for callback function when destination reached
// @note will be called when the destination has been reached
typedef void (*travCtrl_destReached)(void);

/**
 * @brief   To start the whole control.
 * @parameter [in] destReachedCallback Function pointer to callback for when destination is reached
 * @return function pointer to callback for everytime direction angle needs to be updated
*/
travCtrl_dirAngleCb_t travCtrl_init(travCtrl_destReached travCtrl_destReachedCallback);

/**
 * @brief   To start or stop moving.
 * @parameter [in] startGoing true or false to start or stop
*/
void travCtrl_startStop(bool startGoing){


/*===========================================================================*/
/* Functions for testing              */
/*===========================================================================*/
void travCtrl_testAll(void);


#endif /* TRAVELCONTROLLER_H_ */
