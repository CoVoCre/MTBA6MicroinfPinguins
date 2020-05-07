/*
 * travelController.h
 *
 *  Created on: May 7, 2020
 *  Authors: Nicolaj Schmid & Théophane Mayaud
 * 	Project: EPFL MT BA6 penguins epuck2 project
 *
 * Introduction: This file deals with the control of the motors from the direction to be reached,
 * 		and stops when an obstacle/the objective is reached (detection with proximity sensor).
 */

#ifndef TRAVELCONTROLLER_H_
#define TRAVELCONTROLLER_H_

/*
 * @brief type for callback function when obstacle is reached
 * @note is used to declare a pointer to a function that will be called when an obstacle has been reached.
 */
typedef void (*travCtrl_obstacleReached)(void);

/*===========================================================================*/
/* Public functions															*/
/*===========================================================================*/

/*
 * @brief   To start the whole controller, for later moving towards destination.
 *
 * @parameter[in] obstacleReachedCallBackPointer callback for when an obstacle
 * 						is reached and the robot stops.
*/
void travCtrl_init(travCtrl_obstacleReached obstacleReachedCallBackPointer);

/*
 * @brief   Sets the robot moving towards provided angle, or if already moving
 * 				it will just update the direction of movement
 *
 * @parameter[in] directionAngle 	direction to go to, between -180° and 180°
*/
void travelCtrl_goToAngle(int16_t directionAngle);

/*
 * @brief   stops all movements until a new angle is given with travelCtrl_goToAngle
*/
void travCtrl_stopMoving(void);

/*
 * @brief   moves backwards after a source was reached
*/
void travCtrl_moveBackwards(void);

#endif /* TRAVELCONTROLLER_H_ */
