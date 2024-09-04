/*
 * dcm.c
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#include "dcm.h"



void dcm_from_euler( double DCM[][3], const double *euler )
{
	double cos0 = cos( euler[0] ); double sin0 = sin( euler[0] );
	double cos1 = cos( euler[1] ); double sin1 = sin( euler[1] );
	double cos2 = cos( euler[2] ); double sin2 = sin( euler[2] );

	// Inertia to Body
	DCM[0][0] = cos1 * cos2;
	DCM[1][0] = sin0 * sin1 * cos2 - cos0 * sin2;
	DCM[2][0] = cos0 * sin1 * cos2 + sin0 * sin2;

	DCM[0][1] = cos1 * sin2;
	DCM[1][1] = sin0 * sin1 * sin2 + cos0 * cos2;
	DCM[2][1] = cos0 * sin1 * sin2 - sin0 * cos2;

	DCM[0][2] = -sin1;
	DCM[1][2] = sin0 * cos1;
	DCM[2][2] = cos0 * cos1;
}


void dcm_from_quatr( double DCM[][3], const double *quatr )
{
	// Inertia to Body
	DCM[0][0] = 1 - ( quatr[1] * quatr[1] + quatr[2] * quatr[2] ) * 2;
	DCM[1][0] = 2 * ( quatr[0] * quatr[1] - quatr[3] * quatr[2] );
	DCM[2][0] = 2 * ( quatr[0] * quatr[2] + quatr[3] * quatr[1] );

	DCM[0][1] = 2 * ( quatr[1] * quatr[0] + quatr[3] * quatr[2] );
	DCM[1][1] = 1 - ( quatr[0] * quatr[0] + quatr[2] * quatr[2] ) * 2;
	DCM[2][1] = 2 * ( quatr[1] * quatr[2] - quatr[3] * quatr[0] );

	DCM[0][2] = 2 * ( quatr[2] * quatr[0] - quatr[3] * quatr[1] );
	DCM[1][2] = 2 * ( quatr[2] * quatr[1] + quatr[3] * quatr[0] );
	DCM[2][2] = 1 - ( quatr[0] * quatr[0] + quatr[1] * quatr[1] ) * 2;
}


void euler_to_quatr( double *quatr, const double *euler )
{
	double cos0 = cos( euler[0] ); double sin0 = sin( euler[0] );
	double cos1 = cos( euler[1] ); double sin1 = sin( euler[1] );
	double cos2 = cos( euler[2] ); double sin2 = sin( euler[2] );

	quatr[0] =  sin0 * cos1 * cos2 + cos0 * sin1 * sin2;
	quatr[1] = -sin0 * cos1 * sin2 + cos0 * sin1 * cos2;
	quatr[2] =  sin0 * sin1 * cos2 + cos0 * cos1 * sin2;
	quatr[3] = -sin0 * sin1 * sin2 + cos0 * cos1 * cos2;
}
