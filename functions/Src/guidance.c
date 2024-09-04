/*
 * guidance.c
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#include "guidance.h"



st_Guidance Guidance;


void Init_Guidance( st_Guidance *Guidance )
{
	Guidance->w_h = 0.600;
	Guidance->j_h = 1.414;

	Guidance->w_v = 0.600;
	Guidance->j_v = 2.000;

	Guidance->Kp_v = 1 * Guidance->w_v * Guidance->w_v;
	Guidance->Kd_v = 2 * Guidance->w_v * Guidance->j_v;

	Guidance->Kp_h = 1 * Guidance->w_h * Guidance->w_h;
	Guidance->Kd_h = 2 * Guidance->w_h * Guidance->j_h;

	Guidance->max_ATT = 5.0 * D2R;

	Guidance->Target_N =  2.;
	Guidance->Target_E =  0.;
	Guidance->Target_D = -4.;

	Guidance->PhasingTime = 0.;
	Guidance->t = 0.;
	Guidance->T = 1.;

	Guidance->Hz = 10.;
	Guidance->dt = 1. / Guidance->Hz;

	Guidance->GuidancePhase = Guidance_Suspend;
}


void Smooth_Command( double *Target, st_Guidance *Guidance )
{
	double T = Guidance->T;
	double t = Guidance->t - Guidance->PhasingTime;

	double* PhasingPosition = Guidance->PhasingPosition;
	double* TargetPosition  = Guidance->TargetPosition;

	if ( t < T )
	{
		TargetPosition[0] = PhasingPosition[0] + ( Target[0] - PhasingPosition[0] ) / ( 2 * T * T ) * ( t * t );
		TargetPosition[1] = PhasingPosition[1] + ( Target[1] - PhasingPosition[1] ) / ( 2 * T * T ) * ( t * t );
		TargetPosition[2] = PhasingPosition[2] + ( Target[2] - PhasingPosition[2] ) / ( 2 * T * T ) * ( t * t );
	}
	else if ( t < 2 * T )
	{
		TargetPosition[0] = Target[0] - ( Target[0] - PhasingPosition[0] ) / ( 2 * T * T ) * ( t - 2 * T ) * ( t - 2 * T );
		TargetPosition[1] = Target[1] - ( Target[1] - PhasingPosition[1] ) / ( 2 * T * T ) * ( t - 2 * T ) * ( t - 2 * T );
		TargetPosition[2] = Target[2] - ( Target[2] - PhasingPosition[2] ) / ( 2 * T * T ) * ( t - 2 * T ) * ( t - 2 * T );
	}
	else
	{
		TargetPosition[0] = Target[0];
		TargetPosition[1] = Target[1];
		TargetPosition[2] = Target[2];
	}

	Guidance->t += Guidance->dt;
}


void Do_Guidance( st_Guidance *Guidance, const st_Rocket *Rocket )
{
	double cos2 = cos( Rocket->ATT[2] );
	double sin2 = sin( Rocket->ATT[2] );

	double AccMag;
	double Thrust;

	double max_ATT = Guidance->max_ATT;

	double MaxThrust = Actuator.MaxThrust;

	double temp0[3];
	double temp1[3];

	double TargetPosition[3];

	double Command[4];

	// Guidance Phasing for Waypoint
	if ( Guidance->GuidancePhase == Guidance_TakeOff )
	{
		// Convert or not to TakeOff
		if ( Rocket->POS[2] > Guidance->Target_D * 0.95 ) Guidance->GuidancePhase = Guidance_TakeOff;
		else
		{
			Guidance->GuidancePhase = Guidance_HorMove;

			Guidance->PhasingPosition[0] = Rocket->POS[0];
			Guidance->PhasingPosition[1] = Rocket->POS[1];
			Guidance->PhasingPosition[2] = Rocket->POS[2];

			Guidance->PhasingTime = Guidance->t;
		}
	}
	else if ( Guidance->GuidancePhase == Guidance_HorMove )
	{
		// Convert or not to Horizontal Move
		if ( Rocket->POS[0] < Guidance->Target_N * 0.95 ) Guidance->GuidancePhase = Guidance_HorMove;
		else
		{
			Guidance->GuidancePhase = Guidance_Landing;

			Guidance->PhasingPosition[0] = Rocket->POS[0];
			Guidance->PhasingPosition[1] = Rocket->POS[1];
			Guidance->PhasingPosition[2] = Rocket->POS[2];

			Guidance->PhasingTime = Guidance->t;
		}
	}
	else if ( Guidance->GuidancePhase == Guidance_Landing )
	{
		// Convert or not ot Horizontal Move
		if ( Rocket->POS[2] < 0 ) Guidance->GuidancePhase = Guidance_Landing;
		else
		{
			Guidance->GuidancePhase = Guidance_END;

			Guidance->PhasingPosition[0] = Rocket->POS[0];
			Guidance->PhasingPosition[1] = Rocket->POS[1];
			Guidance->PhasingPosition[2] = Rocket->POS[2];

			Guidance->PhasingTime = Guidance->t;
		}
	}
	else if ( Guidance->GuidancePhase == Guidance_Suspend )
	{
		// Suspend
		Guidance->GuidancePhase = Guidance_TakeOff;

		Guidance->PhasingPosition[0] = Rocket->POS[0];
		Guidance->PhasingPosition[1] = Rocket->POS[1];
		Guidance->PhasingPosition[2] = Rocket->POS[2];

		Guidance->PhasingTime = Guidance->t;
	}

	// Guidance Phasing for Waypoint
	if ( Guidance->GuidancePhase == Guidance_TakeOff )
	{
		TargetPosition[0] = Rocket->POS[0];
		TargetPosition[1] = Rocket->POS[1];
		TargetPosition[2] = Guidance->Target_D;
	}
	else if ( Guidance->GuidancePhase == Guidance_HorMove )
	{
		TargetPosition[0] = Guidance->Target_N;
		TargetPosition[1] = Guidance->Target_E;
		TargetPosition[2] = Guidance->Target_D;
	}
	else if ( Guidance->GuidancePhase == Guidance_Landing )
	{
		TargetPosition[0] = Rocket->POS[0];
		TargetPosition[1] = Rocket->POS[1];
		TargetPosition[2] = 0.;
	}
	else if ( Guidance->GuidancePhase == Guidance_END )
	{
		TargetPosition[0] = 0.;
		TargetPosition[1] = 0.;
		TargetPosition[2] = 0.;
	}
	else
	{
		TargetPosition[0] = Guidance->Target_N;
		TargetPosition[1] = Guidance->Target_E;
		TargetPosition[2] = Guidance->Target_D;
	}

	Smooth_Command( TargetPosition, Guidance );

	// PD loop to Calculate Acceleration Command
	temp0[0] = Guidance->Kp_h * ( Guidance->TargetPosition[0] - Rocket->POS[0] ) - Guidance->Kd_h * ( Rocket->VEL[0] );
	temp0[1] = Guidance->Kp_h * ( Guidance->TargetPosition[1] - Rocket->POS[1] ) - Guidance->Kd_h * ( Rocket->VEL[1] );
	temp0[2] = Guidance->Kp_v * ( Guidance->TargetPosition[2] - Rocket->POS[2] ) - Guidance->Kd_v * ( Rocket->VEL[2] ) - 9.81;

	temp1[0] = cos2 * temp0[0] + sin2 * temp0[1];
	temp1[1] = cos2 * temp0[1] - sin2 * temp0[0];
	temp1[2] = temp0[2];

	// Convert Acceleration Command into Thrust and Attitude Command
	AccMag = sqrt( pow( temp1[0], 2 ) + pow( temp1[1], 2 ) + pow( temp1[2], 2 ) );
	Thrust = Rocket->Mass * AccMag;

	// Thrust Saturation
	if ( Thrust > MaxThrust )
	{
		double invThrust = 1. / Thrust;

		Guidance->AccCommand[0] = temp1[0] * MaxThrust * invThrust;
		Guidance->AccCommand[1] = temp1[1] * MaxThrust * invThrust;
		Guidance->AccCommand[2] = temp1[2] * MaxThrust * invThrust;
	}
	else
	{
		Guidance->AccCommand[0] = temp1[0];
		Guidance->AccCommand[1] = temp1[1];
		Guidance->AccCommand[2] = temp1[2];
	}

	AccMag = sqrt( pow( Guidance->AccCommand[0], 2 ) + pow( Guidance->AccCommand[1], 2 ) + pow( Guidance->AccCommand[2], 2 ) );
	Thrust = Rocket->Mass * AccMag;

	Command[0] =  atan2( Guidance->AccCommand[1], sqrt( pow( Guidance->AccCommand[0], 2 ) + pow( Guidance->AccCommand[2], 2 ) ) );
	Command[1] = -atan2( Guidance->AccCommand[0], -Guidance->AccCommand[2] );
	Command[2] =  Rocket->ATT[2];
	Command[3] =  Thrust;

	if ( Command[0] > max_ATT )
	{
		Command[0] = max_ATT;
	}
	else if ( Command[0] < -max_ATT )
	{
		Command[0] = -max_ATT;
	}

	if ( Command[1] > max_ATT )
	{
		Command[1] = max_ATT;
	}
	else if ( Command[1] < -max_ATT )
	{
		Command[1] = -max_ATT;
	}

	Guidance->Command[0] = Command[0];
	Guidance->Command[1] = Command[1];
	Guidance->Command[2] = Command[2];
	Guidance->Command[3] = Command[3];
}
