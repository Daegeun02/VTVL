/*
 * force_moment.c
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#include "force_moment.h"



st_ForceMoment ForceMoment;


void Cal_ForceMoment( st_ForceMoment *ForceMoment, const st_Actuator *Actuator, const st_Rocket *Rocket )
{
	double gravity[3];

	gravity[0] = Rocket->DCM[0][2] * Rocket->Mass * 9.81;
	gravity[1] = Rocket->DCM[1][2] * Rocket->Mass * 9.81;
	gravity[2] = Rocket->DCM[2][2] * Rocket->Mass * 9.81;

	ForceMoment->ForceA[0] = -Actuator->Thrust * Actuator->theta[0];
	ForceMoment->ForceA[1] = -Actuator->Thrust * Actuator->theta[1];
	ForceMoment->ForceA[2] = -Actuator->Thrust;

	ForceMoment->ForceB[0] = gravity[0] + ForceMoment->ForceA[0];
	ForceMoment->ForceB[1] = gravity[1] + ForceMoment->ForceA[1];
	ForceMoment->ForceB[2] = gravity[2] + ForceMoment->ForceA[2];

	ForceMoment->Moment[0] = Rocket->Moment_Arm[1] * ForceMoment->ForceA[2] - Rocket->Moment_Arm[2] * ForceMoment->ForceA[1];
	ForceMoment->Moment[1] = Rocket->Moment_Arm[2] * ForceMoment->ForceA[0] - Rocket->Moment_Arm[0] * ForceMoment->ForceA[2];
	ForceMoment->Moment[2] = Rocket->Moment_Arm[0] * ForceMoment->ForceA[1] - Rocket->Moment_Arm[1] * ForceMoment->ForceA[0];

	// roll
}
