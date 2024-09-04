/*
 * imu.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>


typedef struct {
	double ACC[3];
	double ANV[3];
	double ATT[3];
	double QUT[4];
	double MAG[3];

	uint32_t COUNT;
} st_IMU;

extern st_IMU IMU;

#endif /* INC_IMU_H_ */
