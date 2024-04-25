/*
 * motortest.h
 *
 *  Created on: Apr 25, 2024
 *      Author: Jonas
 */

#ifndef INC_MOTORTEST_H_
#define INC_MOTORTEST_H_

#include "stm32g4xx_hal.h"


void motorTimInit(TIM_HandleTypeDef *htim);
void moveBackwards(uint8_t speedPercent);
void moveForwards(uint8_t speedPercent);
void stopMotor();
void Calibrate();
void Straighten();
void TurnLeft();
void TurnRight();


#endif /* INC_MOTORTEST_H_ */
