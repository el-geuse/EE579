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
void refreshAPDSData();
uint16_t getLidarDistance();
uint16_t percentageToTIM3(uint32_t percentage) ;
uint16_t percentageToTIM4(uint32_t percentage) ;
uint16_t distanceToPercentage(uint32_t distance);
uint16_t getIrLeftDistance();
uint16_t getIrRightDistance();

#endif /* INC_SENSORS_H_ */
