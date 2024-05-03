#include "stm32g4xx_hal.h"
#include "motortest.h"

TIM_HandleTypeDef *htimMotor;
TIM_HandleTypeDef *htimSmack;

void motorTimInit(TIM_HandleTypeDef *htim)
{
	htimMotor = htim;
	HAL_TIM_PWM_Start(htimMotor, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htimMotor, TIM_CHANNEL_3);
	stopMotor();
}

void smackInit(TIM_HandleTypeDef *htim)
{
	TIM4->CCR2 = 500;
	HAL_TIM_PWM_Start(htimSmack, TIM_CHANNEL_2);
	htimSmack = htim;
	TIM4->CCR2 = 0;
}

void moveBackwards(uint8_t speedPercent) {
    if (speedPercent > 100) speedPercent = 100; // Limit speed to 100%
    TIM3->CCR2 = 0;                             // Stop PWM Channel 2
    TIM3->CCR1 = (1000 / 100) * speedPercent; // Calculate CCR value
    HAL_TIM_PWM_Start(htimMotor, TIM_CHANNEL_1); // Start PWM on channel 1
}

void moveForwards(uint8_t speedPercent) {
    if (speedPercent > 100) speedPercent = 100; // Limit speed to 100%
    TIM3->CCR1 = 0;                             // Stop PWM Channel 1
    TIM3->CCR2 = (1000 / 100) * speedPercent; // Calculate CCR value
    HAL_TIM_PWM_Start(htimMotor, TIM_CHANNEL_2); // Start PWM on channel 2
}


void stopMotor() {
    TIM3->CCR1 = 0; // Stop channel 1 by setting duty cycle to 0
    TIM3->CCR2 = 0; // Stop channel 2 by setting duty cycle to 0
}

void calibrate(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void straighten(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

}

void turnLeft(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

}

void turnRight(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

}

void smack(){
	TIM4->CCR2 = 2500;
    HAL_Delay(2000);
    TIM4->CCR2 = 500;
    HAL_Delay(2000);
    TIM4->CCR2 = 0;
}



