/*
 * gps.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdint.h>



typedef struct {
	double LLH[3];
	double ALT;
	double Hor_Acu;
	double Ver_Acu;

	double VEL[3];
	double SPD;
	double HDG;
	double Ground_SPD;
	double SPD_Acu;
	double HDG_Acu;

	uint8_t Fix;

	uint16_t LLH_Validity;
	uint16_t SPG_Validity;
	uint16_t Fix_Validity;

	uint16_t SatNum;

	uint32_t COUNT;
} st_GPS;

extern st_GPS GPS;

#endif /* INC_GPS_H_ */
