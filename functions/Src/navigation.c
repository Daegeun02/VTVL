/*
 * navigation.c
 *
 *  Created on: Jun 19, 2024
 *      Author: daegeun
 */

#include <navigation.h>



void Do_NED_to_LLH( double *LLH, const double *NED, const double *LL0 )
{
	double temp[3];
	double ECEF[3];
	double ECEF2NED[3][3];

	double Rt;
	double alt_old;

	temp[0] =  0.;
	temp[1] = -LL0[0] - ( PI / 2. );
	temp[2] =  LL0[1];

	dcm_from_euler( ECEF2NED, temp );

	// LL0 Position into ECEF
	Do_LLH_to_ECEF( ECEF, LL0 );

	// NED Position into ECEF
	ECEF[0] += ECEF2NED[0][0] * NED[0] + ECEF2NED[1][0] * NED[1] + ECEF2NED[2][0] * NED[2];
	ECEF[1] += ECEF2NED[0][1] * NED[0] + ECEF2NED[1][1] * NED[1] + ECEF2NED[2][1] * NED[2];
	ECEF[2] += ECEF2NED[0][2] * NED[0] + ECEF2NED[1][2] * NED[1] + ECEF2NED[2][2] * NED[2];

	// Convert ECEF into LLH

	temp[0] = 0.;
	temp[1] = sqrt( ECEF[0]*ECEF[0] + ECEF[1]*ECEF[1] );
	temp[2] = 0.;

	Rt      = Re;
	alt_old = 1.e8;

	while ( _abs( temp[2] - alt_old ) > 1e-6 )
	{
		alt_old = temp[2];

		temp[0] = atan2( ECEF[2], temp[1] * ( 1 - ( ( e * e * Rt ) / ( Rt + temp[2] ) ) ) );
		Rt      = Re / sqrt( 1 - ( e * sin( temp[0] ) ) * ( e * sin( temp[0]) ) );
		temp[2] = ( temp[1] / cos( temp[0] ) ) - Rt;
	}

	temp[1] = atan2( ECEF[1], ECEF[0] );

	LLH[0] = temp[0];
	LLH[1] = temp[1];
	LLH[2] = temp[2];
}


void Do_LLH_to_ECEF( double *ECEF, const double *LLH )
{
	double cos_Lat = cos( LLH[0] );
	double sin_Lat = sin( LLH[0] );
	double cos_Lon = cos( LLH[1] );
	double sin_Lon = sin( LLH[1] );

	double Rt = Re / sqrt( 1 - ( e * sin_Lat ) * ( e * sin_Lat ) );

	ECEF[0] = ( Rt + LLH[2] ) * cos_Lat * cos_Lon;
	ECEF[1] = ( Rt + LLH[2] ) * cos_Lat * sin_Lon;
	ECEF[2] = ( Rt * ( 1 - e*e ) + LLH[2] ) * sin_Lat;
}


void Do_LLH_to_NED( double *NED, const double *LLH, const double *LL0 )
{
	double temp[3];

	double ECEF1[3];
	double ECEF2[3];

	double ECEF2NED[3][3];

	Do_LLH_to_ECEF( ECEF1, LL0 );
	Do_LLH_to_ECEF( ECEF2, LLH );

	temp[0] =  0.;
	temp[1] = -LL0[0] - ( PI / 2. );
	temp[2] =  LL0[1];

	dcm_from_euler( ECEF2NED, temp );

	ECEF2[0] -= ECEF1[0];
	ECEF2[1] -= ECEF1[1];
	ECEF2[2] -= ECEF1[2];

	NED[0] = ECEF2NED[0][0] * ECEF2[0] + ECEF2NED[0][1] * ECEF2[1] + ECEF2NED[0][2] * ECEF2[2];
	NED[1] = ECEF2NED[1][0] * ECEF2[0] + ECEF2NED[1][1] * ECEF2[1] + ECEF2NED[1][2] * ECEF2[2];
	NED[2] = ECEF2NED[2][0] * ECEF2[0] + ECEF2NED[2][1] * ECEF2[1] + ECEF2NED[2][2] * ECEF2[2];
}


double _abs( double value )
{
	double output;

	if ( value > 0 )
	{
		output = value;
	}
	else
	{
		output = -value;
	}

	return output;
}
