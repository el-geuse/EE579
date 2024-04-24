/*
 * data_lut.h
 *
 *  Created on: 23.04.2024
 *      Author: tizia
 */

#ifndef INC_DATA_LUT_H_
#define INC_DATA_LUT_H_

typedef struct { double x; double y; } coord_t;
double interp(coord_t *c, double x, int n);

coord_t lut_GP2Y0A02YK0F[15] =
{
		{0.4,150},
		{0.45,140},
		{0.5,130},
		{0.55,120},
		{0.6,110},
		{0.65,100},
		{0.7,90},
		{0.8,80},
		{0.9,70},
		{1.1,60},
		{1.5,50},
		{1.55,40},
		{1.95,30},
		{2.55,20},
		{2.75,15}

};

#endif /* INC_DATA_LUT_H_ */
