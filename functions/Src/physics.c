/*
 * physics.c
 *
 *  Created on: Jun 12, 2024
 *      Author: daegeun
 */

#include "physics.h"


void Do_PhysicalUpdate( st_Rocket *Rocket, st_Actuator *Actuator )
{
	// Mass Estimate

	// Inertia Estimate

	// Convert attitude into quaternion
	euler_to_quatr( Rocket->QUT, Rocket->ATT );

	// Convert Position NED into Global LLH
	Do_NED_to_LLH( Rocket->LLA, Rocket->POS, Rocket->LL0 );
}
