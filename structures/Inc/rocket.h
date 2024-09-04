/*
 * rocket.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_ROCKET_H_
#define INC_ROCKET_H_

#include <constants.h>


typedef struct {
	double V_B[3];
	double VEL[3];
	double POS[3];

	double ACC[3];
	double ANV[3];
	double ATT[3];
	double LL0[3];
	double LLA[3];
	double QUT[4];
	double RPS[3];
	double yaw;
	double spd;

	double DCM[3][3];

	double Mass;

	double Inertia[3][3];

	double Moment_Arm[3];

	double atmalt;
	double ldralt;
} st_Rocket;


extern st_Rocket Rocket;

void Init_Rocket( st_Rocket *Rocket );

#endif /* INC_ROCKET_H_ */
