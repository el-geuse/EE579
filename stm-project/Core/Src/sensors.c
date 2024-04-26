/*
 * sensors.c
 *
 *  Created on: Feb 23, 2024
 *      Author: Jonas
 */

#include "stm32g4xx_hal.h"
#include "data_lut.h"

#define leftIR_GP2Y0A02YK0F
//#define leftIR_GP2Y0A41SK0F
#define rightIR_GP2Y0A02YK0F
//#define rightIR_GP2Y0A41SK0F

uint16_t clear, red, green, blue, apdsDistance;
I2C_HandleTypeDef *i2c;
ADC_HandleTypeDef *hadc_ir;


void sensorsAdcInit(ADC_HandleTypeDef *hadc)
{
	hadc_ir = hadc;
}

void sensorsI2CInit(I2C_HandleTypeDef *hi2c)
{
	i2c = hi2c;
}

void initAPDS()
{
	uint8_t debug;
	uint8_t sent[] = {0x80, 0x07};
	uint8_t sent2[] = {0x8F, 0x0F};

	debug = HAL_I2C_Master_Transmit(i2c, (uint8_t)0x39*2, sent, 2, 1000);
	debug = HAL_I2C_Master_Transmit(i2c, (uint8_t)0x39*2, sent2, 2, 1000);
}

uint16_t getLidarDistance()
{
	uint8_t sent[] = {0x00};
	uint8_t receive[4] = {0};
	uint8_t comError = 0;
	uint16_t distance;


	if (HAL_I2C_Master_Transmit(i2c, (uint8_t)0x20, sent, 1, 1000)
	 || HAL_I2C_Master_Receive(i2c, (uint8_t)0x21, receive, 4, 1000))
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

uint16_t fetchAPDSData(I2C_HandleTypeDef *i2c, uint8_t *data)
{
	uint8_t sent[] = {0x93};
	uint8_t comError = 0;

	if (HAL_I2C_Master_Transmit(i2c, (uint8_t)0x39*2, sent, 1, 1000)
	 || HAL_I2C_Master_Receive(i2c, (uint8_t)0x39*2+1, data, 10, 1000))
	{
		comError = 1;
	}

	if (comError) {
		clear = 0;
	}


	return clear;
}

uint16_t percentageToTIM3(uint32_t percentage) {
	return (uint32_t)500 * percentage / 100;
}

uint16_t percentageToTIM4(uint32_t percentage) {
	return (uint32_t)20000 * percentage / 100;
}

uint16_t distanceToPercentage(uint32_t distance) {
	if (distance > 105)
		distance = 100;
	if (distance < 005)
		distance = 005;
	distance = distance - 5;

	return ((uint32_t)100 - distance) * ((uint32_t)100 - distance) *((uint32_t)100 - distance) / 10000;
}

void refreshAPDSData()
{
	uint8_t buf[10];
	fetchAPDSData(i2c, buf);
	clear = buf[2] << 8 | buf[1];
	red = buf[4] << 8 | buf[3];
	green = buf[6] << 8 | buf[5];
	blue = buf[8] << 8 | buf[7];
	apdsDistance = buf[9];
}

uint16_t getIrLeftDistance()
{
	uint16_t voltage, distance;
	HAL_ADC_Start(hadc_ir);
	HAL_ADC_PollForConversion(hadc_ir, 1000);
	voltage = HAL_ADC_GetValue(hadc_ir);
	HAL_ADC_Start(hadc_ir);
	HAL_ADC_PollForConversion(hadc_ir, 1000);

#ifdef leftIR_GP2Y0A41SK0F
	distance = (float)13*4096 / voltage - 0.5;
#endif
#ifdef leftIR_GP2Y0A02YK0F
	distance = interp(lut_GP2Y0A02YK0F, (float)voltage*3.3/4096, 15);
#endif
	return distance;
}
uint16_t getIrRightDistance()
{
	uint16_t voltage, distance;
	HAL_ADC_Start(hadc_ir);
	HAL_ADC_PollForConversion(hadc_ir, 1000);

	HAL_ADC_Start(hadc_ir);
	HAL_ADC_PollForConversion(hadc_ir, 1000);

	voltage = HAL_ADC_GetValue(hadc_ir);

#ifdef rightIR_GP2Y0A41SK0F
	distance = (float)13*4096 / voltage - 0.5;
#endif
#ifdef rightIR_GP2Y0A02YK0F
	distance = interp(lut_GP2Y0A02YK0F, (float)voltage*3.3/4096, 15);
#endif


	return distance;

}
