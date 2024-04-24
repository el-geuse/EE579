/*
 * sensors.h
 *
 *  Created on: 23.04.2024
 *      Author: tizia
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

void sensorsAdcInit(ADC_HandleTypeDef *hadc);
void sensorsI2CInit(I2C_HandleTypeDef *hi2c);
void initAPDS();
uint16_t getLidarDistance();
uint32_t percentageToTIM3(uint32_t percentage) ;
uint32_t distanceToPercentage(uint32_t distance);
void getAPDSData();
uint16_t getIrLeftDistance();
uint16_t getIrRightDistance();

//Private
//int16_t fetchAPDSData(I2C_HandleTypeDef *i2c, uint8_t *data);

#endif /* INC_SENSORS_H_ */
