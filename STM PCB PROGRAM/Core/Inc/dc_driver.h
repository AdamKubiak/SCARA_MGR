/*
 * dc_driver.h
 *
 *  Created on: 28 mar 2023
 *      Author: Adum
 */

#ifndef INC_DC_DRIVER_H_
#define INC_DC_DRIVER_H_

void moveStopJoint1();
void moveStopJoint2();
void moveLeftJoint1();
void moveRightJoint1();
void moveLeftJoint2();
void moveRightJoint2();
void setMotorSpeed1(TIM_HandleTypeDef *htim,long int valuePWM);
void setMotorSpeed2(TIM_HandleTypeDef *htim,long int valuePWM);

#endif /* INC_DC_DRIVER_H_ */
