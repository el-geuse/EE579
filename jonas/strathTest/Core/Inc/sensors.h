/*
 * sensors.h
 *
 *  Created on: Feb 23, 2024
 *      Author: tizia
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

uint16_t getLidarDistance(I2C_HandleTypeDef *a);
uint16_t getAPDSData(I2C_HandleTypeDef *a, uint8_t *b);
#endif /* INC_SENSORS_H_ */
