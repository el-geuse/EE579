/*
 * sensors.c
 *
 *  Created on: Feb 23, 2024
 *      Author: Jonas
 */

#include "stm32h5xx_hal.h"

uint16_t getLidarDistance(I2C_HandleTypeDef *i2cHandler)
{
	uint8_t sent[] = {0x00};
	uint8_t receive[4] = {0};
	uint8_t comError = 0;
	uint16_t distance;


	if (HAL_I2C_Master_Transmit(i2cHandler, (uint8_t)0x20, sent, 1, HAL_MAX_DELAY)
	 || HAL_I2C_Master_Receive(i2cHandler, (uint8_t)0x21, receive, 4, HAL_MAX_DELAY))
	{
		comError = 1;
	}

	if (comError == 0) {
		distance = (receive[1] << 8) | receive[0];
	} else {
		distance = 0;
	}

	return distance;
}

uint16_t getAPDSData(I2C_HandleTypeDef *i2cHandler, uint8_t *data)
{
	uint8_t sent[] = {0x93};
	uint8_t comError = 0;
	uint16_t clear;

	if (HAL_I2C_Master_Transmit(i2cHandler, (uint8_t)0x39*2, sent, 1, HAL_MAX_DELAY)
	 || HAL_I2C_Master_Receive(i2cHandler, (uint8_t)0x39*2+1, data, 10, HAL_MAX_DELAY))
	{
		comError = 1;
	}

	if (comError == 0) {
		clear = (data[2] << 8) | data[1];
	} else {
		clear = 0;
	}

	return clear;
}

