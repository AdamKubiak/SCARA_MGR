/*
 * dc_driver.c
 *
 *  Created on: 28 mar 2023
 *      Author: Adum
 */
#include "stm32f4xx.h"
#include "stdlib.h"

void moveStopJoint1()
{
	//prawo góra forward
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
}

void moveStopJoint2()
{
	//prawo góra forward
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
}


void moveLeftJoint1()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
}

void moveLeftJoint2()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
}


void moveRightJoint1()
{

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
}

void moveRightJoint2()
{

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
}

void setMotorSpeed1(TIM_HandleTypeDef *htim,long int valuePWM)
{
	if(valuePWM>70)
	{
		valuePWM = 70;
	}
	else if(valuePWM<-70)
	{
		valuePWM = -70;
	}

	if(valuePWM >= 0)
	{
		moveLeftJoint1();
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,valuePWM);
	}
	else
	{
		moveRightJoint1();
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,abs(valuePWM));
	}
}

void setMotorSpeed2(TIM_HandleTypeDef *htim,long int valuePWM)
{
	if(valuePWM>70)
	{
		valuePWM = 70;
	}
	else if(valuePWM<-70)
	{
		valuePWM = -70;
	}

	if(valuePWM >= 0)
	{
		moveLeftJoint2();
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,valuePWM);
	}
	else
	{
		moveRightJoint2();
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,abs(valuePWM));
	}
}
