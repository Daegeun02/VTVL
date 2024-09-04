/*
 * actuator.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include "actuator.h"



st_Actuator Actuator;


void Init_Actuator( st_Actuator *Actuator )
{
	Actuator->TVC_1 = TVC_NEUT_1;
	Actuator->TVC_2 = TVC_NEUT_2;

	Actuator->w = 80.;
	Actuator->j = 0.7;

	Actuator->Kp = 1 * Actuator->w * Actuator->w;
	Actuator->Kd = 2 * Actuator->w * Actuator->j;

	Actuator->dY0[0] = 0.;
	Actuator->dY0[1] = 0.;

	Actuator->Y0[0] = 0.;
	Actuator->Y0[1] = 0.;

	Actuator->dY1[0] = 0.;
	Actuator->dY1[1] = 0.;

	Actuator->Y1[0] = 0.;
	Actuator->Y1[1] = 0.;

	Actuator->Hz = 400.;
	Actuator->dt = 1. / Actuator->Hz;

	Actuator->theta[0] = 0.;
	Actuator->theta[1] = 0.;
	/* TVC End */

	/* Engine Start */
	Actuator->tau     = 1.1;
	Actuator->tau_inv = 1. / Actuator->tau;

	Actuator->MaxThrust = 450.;

	Actuator->Thrust  = 300.;
	Actuator->dThrust = 0.;

	Actuator->ThrustLevel=100.;

	Actuator->PWM = 1000;
	Actuator->RPM = 0.;
	/* Engine End */
}


void Do_Actuator( st_Actuator *Actuator, const st_Controller *Controller )
{
	double dt = Actuator->dt;

	/* TVC Start */
	Actuator->dY0[0] = Actuator->Y0[1];
	Actuator->dY0[1] = Actuator->Kp * ( Controller->theta[0] - Actuator->Y0[0] ) - Actuator->Kd * ( Actuator->Y0[1] );

	Actuator->dY1[0] = Actuator->Y1[1];
	Actuator->dY1[1] = Actuator->Kp * ( Controller->theta[1] - Actuator->Y1[0] ) - Actuator->Kd * ( Actuator->Y1[1] );

	Actuator->Y0[0] += Actuator->dY0[0] * dt;
	Actuator->Y0[1] += Actuator->dY0[1] * dt;

	Actuator->Y1[0] += Actuator->dY1[0] * dt;
	Actuator->Y1[1] += Actuator->dY1[1] * dt;

	Actuator->theta[0] = Actuator->Y0[0];
	Actuator->theta[1] = Actuator->Y1[0];
	/* TVC End */

	/* Engine Start */
	Actuator->dThrust = Actuator->tau_inv * ( Controller->Thrust - Actuator->Thrust );

	Actuator->Thrust += Actuator->dThrust * dt;
	/* Engine End */
}
