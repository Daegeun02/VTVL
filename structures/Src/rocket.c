/*
 * rocket.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include "rocket.h"


st_Rocket Rocket;


void Init_Rocket( st_Rocket *Rocket )
{
	Rocket->VEL[0] = 0.0;
	Rocket->VEL[1] = 0.0;
	Rocket->VEL[2] = 0.0;

	Rocket->POS[0] = 0.0;
	Rocket->POS[1] = 0.0;
	Rocket->POS[2] = 0.0;

	Rocket->ACC[0] = 0.0;
	Rocket->ACC[1] = 0.0;
	Rocket->ACC[2] = 0.0;

	Rocket->ANV[0] = 0.0;
	Rocket->ANV[1] = 0.0;
	Rocket->ANV[2] = 0.0;

	Rocket->ATT[0] = 0.0;
	Rocket->ATT[1] = 0.0;
	Rocket->ATT[2] = 0.0;

	Rocket->LL0[0] =  37.45132396929280 * D2R;
	Rocket->LL0[1] = 126.65028325281600 * D2R;
	Rocket->LL0[2] = 0.0;

	Rocket->LLA[0] = Rocket->LL0[0];
	Rocket->LLA[1] = Rocket->LL0[1];
	Rocket->LLA[2] = Rocket->LL0[2];

	Rocket->QUT[0] = 0.0;
	Rocket->QUT[1] = 0.0;
	Rocket->QUT[2] = 0.0;
	Rocket->QUT[3] = 1.0;

	for ( int i = 0; i < 3; i++ )
	{
		for ( int j = 0; j < 3; j++ )
		{
			Rocket->DCM[i][j] = 0.0;

			Rocket->Inertia[i][j] = 0.0;
		}
	}

	Rocket->Inertia[0][0] = 6.54375;
	Rocket->Inertia[1][1] = 6.54375;
	Rocket->Inertia[2][2] = 0.6;

	Rocket->Mass = 30.;

	Rocket->DCM[0][0] = 1.0;
	Rocket->DCM[1][1] = 1.0;
	Rocket->DCM[2][2] = 1.0;

	Rocket->Moment_Arm[0] = 0.0;
	Rocket->Moment_Arm[1] = 0.0;
	Rocket->Moment_Arm[2] = 0.75;

	Rocket->yaw = 0.0;
	Rocket->spd = 0.0;
}
