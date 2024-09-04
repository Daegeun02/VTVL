/*
 * control.c
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#include "control.h"



st_Controller Controller;



void Init_Controller( st_Controller *Controller, const double Mass )
{
	Controller->TVC_w = 1.000;
	Controller->TVC_j = 0.707;

	Controller->RCS_w = 1.414;
	Controller->RCS_j = 1.000;

	Controller->TVC_Kp = 1 * Controller->TVC_w * Controller->TVC_w;
	Controller->TVC_Kd = 2 * Controller->TVC_w * Controller->TVC_j;

	Controller->RCS_Kp = 1 * Controller->RCS_w * Controller->RCS_w;
	Controller->RCS_Kd = 2 * Controller->RCS_w * Controller->RCS_j;

	Controller->max_theta = 5.0 * D2R;

	Controller->theta[0] = 0.0;
	Controller->theta[1] = 0.0;

	Controller->Thrust = Mass * 9.81;

	Controller->PWM = 1000;

	Controller->TVC_1 = 1000;
	Controller->TVC_2 = 1000;

	Controller->RCS1 = 1000;
	Controller->RCS2 = 1000;
	Controller->RCS3 = 1000;
	Controller->RCS4 = 1000;
}


void Do_Control( st_Controller *Controller, const st_Rocket *Rocket, const double *Command )
{
	double deuler[3][3];

	double temp0[3];
	double temp1[2];
	double max_theta = Controller->max_theta;

	double cos0 = cos( Rocket->ATT[0] );
	double sin0 = sin( Rocket->ATT[0] );
	double tan1 = tan( Rocket->ATT[1] );
	double sec1 = 1. / cos( Rocket->ATT[1] );

	deuler[0][0] = 1.;
	deuler[0][1] = sin0 * tan1;
	deuler[0][2] = cos0 * tan1;

	deuler[1][0] =  0.;
	deuler[1][1] =  cos0;
	deuler[1][2] = -sin0;

	deuler[2][0] = 0.;
	deuler[2][1] = sin0 * sec1;
	deuler[2][2] = cos0 * sec1;

	temp0[0] = deuler[0][0] * Rocket->ANV[0] + deuler[0][1] * Rocket->ANV[1] + deuler[0][2] * Rocket->ANV[2];
	temp0[1] = deuler[1][0] * Rocket->ANV[0] + deuler[1][1] * Rocket->ANV[1] + deuler[1][2] * Rocket->ANV[2];
	temp0[2] = deuler[2][0] * Rocket->ANV[0] + deuler[2][1] * Rocket->ANV[1] + deuler[2][2] * Rocket->ANV[2];

	/* TVC Attitude Control Loop START */
	temp1[0] =  Controller->TVC_Kp * ( Command[0] - Rocket->ATT[0] ) - Controller->TVC_Kd * ( temp0[0] );
	temp1[1] = -Controller->TVC_Kp * ( Command[1] - Rocket->ATT[1] ) + Controller->TVC_Kd * ( temp0[1] );

	// Saturation
	if ( temp1[0] > max_theta )
	{
		temp1[0] = max_theta;
	}
	else if ( temp1[0] < -max_theta )
	{
		temp1[0] = -max_theta;
	}

	if ( temp1[1] > max_theta )
	{
		temp1[1] = max_theta;
	}
	else if ( temp1[1] < -max_theta )
	{
		temp1[1] = -max_theta;
	}

	Controller->theta[0] = temp1[1];
	Controller->theta[1] = temp1[0];
	/* TVC Attitude Control Loop END */

	/* RCS Roll Stability Loop START */
//	temp1[0] =  Controller->RCS_Kp * ( -temp0[2] ) - Controller->RCS_Kd * ( )


	/* RCS Roll Stability Loop END */

	// Mapping

	// Thrust
	Controller->Thrust = Command[3];
}
