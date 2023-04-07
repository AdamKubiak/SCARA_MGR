/*
 * pid.c
 *
 *  Created on: 28 mar 2023
 *      Author: Adum
 */

#include "pid.h"

#define ERR_SUM_MAX	1500

struct pid_params
{
	float proportional;
	float integral;
	float derivative; //k_d = 1
	float controlSignal; //u - Also called as process variable (PV)
	unsigned long long  previousTime; //for calculating delta t
	float previousError; //for calculating the derivative (edot)
	float errorIntegral; //integral error
	float currentTime; //time in the moment of calculation
	float deltaTime; //time difference
	float errorValue; //error
	float edot; //derivative (de/dt)
};

static struct pid_params pid1;
static struct pid_params pid2;

void pid_init1(float kp, float ki, float kd)
{
	pid1.proportional = kp;
	pid1.integral = ki;
	pid1.derivative = kd; //k_d = 1
	pid1.controlSignal = 0; //u - Also called as process variable (PV)
	pid1.previousTime = 0; //for calculating delta t
	pid1.previousError=0; //for calculating the derivative (edot)
	pid1.errorIntegral=0; //integral error
	pid1.currentTime=0; //time in the moment of calculation
	pid1.deltaTime=0; //time difference
	pid1.errorValue=0; //error
	pid1.edot=0; //derivative (de/dt)
}

void pid_init2(float kp, float ki, float kd)
{
	pid2.proportional = kp;
	pid2.integral = ki;
	pid2.derivative = kd; //k_d = 1
	pid2.controlSignal = 0; //u - Also called as process variable (PV)
	pid2.previousTime = 0; //for calculating delta t
	pid2.previousError=0; //for calculating the derivative (edot)
	pid2.errorIntegral=0; //integral error
	pid2.currentTime=0; //time in the moment of calculation
	pid2.deltaTime=0; //time difference
	pid2.errorValue=0; //error
	pid2.edot=0; //derivative (de/dt)
}

void set_pid1(unsigned int kp, unsigned int ki, unsigned int kd)
{
	pid1.proportional = kp/100.;
	pid1.integral = ki/100.;
	pid1.derivative = kd/100.;
}

void set_pid2(unsigned int kp, unsigned int ki, unsigned int kd)
{
	pid2.proportional = kp/100.;
	pid2.integral = ki/100.;
	pid2.derivative = kd/100.;
}


float pid_calculate1(float set_val, float read_val,float currentTime)
{
	pid1.deltaTime = (currentTime - pid1.previousTime) / 1000000.0; //time difference in seconds
	pid1.previousTime = currentTime;//save the current time for the next iteration to get the time difference

	pid1.errorValue = set_val - read_val;

	pid1.edot = (pid1.errorValue - pid1.previousError) / pid1.deltaTime;//edot = de/dt - derivative term

	pid1.errorIntegral = pid1.errorIntegral + (pid1.errorValue * pid1.deltaTime);

	if(pid1.errorIntegral > ERR_SUM_MAX){
		pid1.errorIntegral = ERR_SUM_MAX;
	} else if (pid1.errorIntegral < -ERR_SUM_MAX){
		pid1.errorIntegral = -ERR_SUM_MAX;
	}

	pid1.controlSignal = (pid1.proportional * pid1.errorValue) + (pid1.derivative * pid1.edot) + (pid1.integral * pid1.errorIntegral);

	pid1.previousError = pid1.errorValue;

	return pid1.controlSignal;
}

float pid_calculate2(float set_val, float read_val,float currentTime)
{
	pid2.deltaTime = (currentTime - pid2.previousTime) / 1000000.0; //time difference in seconds
	pid2.previousTime = currentTime;//save the current time for the next iteration to get the time difference

	pid2.errorValue = set_val - read_val;

	pid2.edot = (pid2.errorValue - pid2.previousError) / pid2.deltaTime;//edot = de/dt - derivative term

	pid2.errorIntegral = pid2.errorIntegral + (pid2.errorValue * pid2.deltaTime);

	if(pid2.errorIntegral > ERR_SUM_MAX){
		pid2.errorIntegral = ERR_SUM_MAX;
	} else if (pid2.errorIntegral < -ERR_SUM_MAX){
		pid2.errorIntegral = -ERR_SUM_MAX;
	}

	pid2.controlSignal = (pid2.proportional * pid2.errorValue) + (pid2.derivative * pid2.edot) + (pid2.integral * pid2.errorIntegral);

	pid2.previousError = pid2.errorValue;

	return pid2.controlSignal;
}
