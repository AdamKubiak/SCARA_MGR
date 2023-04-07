/*
 * pid.h
 *
 *  Created on: 28 mar 2023
 *      Author: Adum
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

void set_pid1(unsigned int kp, unsigned int ki, unsigned int kd);
void set_pid2(unsigned int kp, unsigned int ki, unsigned int kd);
void pid_init1(float kp, float ki, float kd);
void pid_init2(float kp, float ki, float kd);
float pid_calculate1(float set_val, float read_val, float currentTime);
float pid_calculate2(float set_val, float read_val,float currentTime);

#endif /* INC_PID_H_ */
