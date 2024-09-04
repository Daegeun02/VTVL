/*
 * dynamic.c
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#include "dynamic.h"



st_State State;


double one_of_six = 1. / 6.;


void Init_State( st_State *State )
{
	for ( int i = 0; i < NumberOfState; i++ )
	{
		State->X[i]  = 0.0;

		State->dX1[i] = 0.0;
		State->dX2[i] = 0.0;
		State->dX3[i] = 0.0;
		State->dX4[i] = 0.0;

		State->dt = 0.0025;
	}

	for ( int i = 0; i < 3; i++ )
	{
		for ( int j = 0; j < 3; j++ )
		{
			State->deuler[i][j] = 0.;
		}
	}

	State->t = 0.;
}


void UploadState( st_State *State, const st_Rocket *Rocket )
{
	double cos0 = cos( Rocket->ATT[0] );
	double sin0 = sin( Rocket->ATT[0] );
	double tan1 = tan( Rocket->ATT[1] );
	double sec1 = 1. / cos( Rocket->ATT[1] );

	State->X[0] = Rocket->POS[0];
	State->X[1] = Rocket->POS[1];
	State->X[2] = Rocket->POS[2];

	State->X[3] = Rocket->ATT[0];
	State->X[4] = Rocket->ATT[1];
	State->X[5] = Rocket->ATT[2];

	State->X[7] = Rocket->V_B[0];
	State->X[8] = Rocket->V_B[1];
	State->X[9] = Rocket->V_B[2];

	State->X[10] = Rocket->ANV[0];
	State->X[11] = Rocket->ANV[1];
	State->X[12] = Rocket->ANV[2];

	State->deuler[0][0] = 1.;
	State->deuler[0][1] = sin0 * tan1;
	State->deuler[0][2] = cos0 * tan1;

	State->deuler[1][0] =  0.;
	State->deuler[1][1] =  cos0;
	State->deuler[1][2] = -sin0;

	State->deuler[2][0] = 0.;
	State->deuler[2][1] = sin0 * sec1;
	State->deuler[2][2] = cos0 * sec1;
}


__inline void dynamic( double *dX, const double *X, const double deuler[][3], const st_Rocket *Rocket, const st_ForceMoment *ForceMoment )
{
	double invmass = 1. / Rocket->Mass;

	dX[ 0] = Rocket->DCM[0][0] * X[7] + Rocket->DCM[1][0] * X[8] + Rocket->DCM[2][0] * X[9];
	dX[ 1] = Rocket->DCM[0][1] * X[7] + Rocket->DCM[1][1] * X[8] + Rocket->DCM[2][1] * X[9];
	dX[ 2] = Rocket->DCM[0][2] * X[7] + Rocket->DCM[1][2] * X[8] + Rocket->DCM[2][2] * X[9];

	dX[ 3] = deuler[0][0] * X[10] + deuler[0][1] * X[11] + deuler[0][2] * X[12];
	dX[ 4] = deuler[1][0] * X[10] + deuler[1][1] * X[11] + deuler[1][2] * X[12];
	dX[ 5] = deuler[2][0] * X[10] + deuler[2][1] * X[11] + deuler[2][2] * X[12];

	dX[ 7] = ForceMoment->ForceB[0] * invmass - ( X[11] * X[9] - X[12] * X[8] );
	dX[ 8] = ForceMoment->ForceB[1] * invmass - ( X[12] * X[7] - X[10] * X[9] );
	dX[ 9] = ForceMoment->ForceB[2] * invmass - ( X[10] * X[8] - X[11] * X[7] );

	dX[10] = ( ForceMoment->Moment[0] + ( Rocket->Inertia[1][1] - Rocket->Inertia[2][2] ) * X[11] * X[12] ) / Rocket->Inertia[0][0];
	dX[11] = ( ForceMoment->Moment[1] + ( Rocket->Inertia[2][2] - Rocket->Inertia[0][0] ) * X[12] * X[10] ) / Rocket->Inertia[1][1];
	dX[12] = ( ForceMoment->Moment[2] + ( Rocket->Inertia[0][0] - Rocket->Inertia[1][1] ) * X[10] * X[11] ) / Rocket->Inertia[2][2];
}


void dxdt( st_State *State, const st_Rocket *Rocket, const st_ForceMoment *ForceMoment, const uint8_t Order )
{
	double temp[13];
	double dt = State->dt;

	switch ( Order ) {

	case 1:
		temp[ 0] = State->X[ 0];	// POS N
		temp[ 1] = State->X[ 1];	// POS E
		temp[ 2] = State->X[ 2];	// POS D

		temp[ 3] = State->X[ 3];	// ATT 0
		temp[ 4] = State->X[ 4];	// ATT 1
		temp[ 5] = State->X[ 5];	// ATT 2
		temp[ 6] = State->X[ 6];	// ATT 3

		temp[ 7] = State->X[ 7];	// VEL X
		temp[ 8] = State->X[ 8];	// VEL Y
		temp[ 9] = State->X[ 9];	// VEL Z

		temp[10] = State->X[10];	// ANV p
		temp[11] = State->X[11];	// ANV Q
		temp[12] = State->X[12];	// ANV R

		dynamic( State->dX1, temp, State->deuler, Rocket, ForceMoment );

	case 2:
		temp[ 0] = State->X[ 0] + (0.5) * dt * State->dX1[ 0];	// POS N
		temp[ 1] = State->X[ 1] + (0.5) * dt * State->dX1[ 1];	// POS E
		temp[ 2] = State->X[ 2] + (0.5) * dt * State->dX1[ 2];	// POS D

		temp[ 3] = State->X[ 3] + (0.5) * dt * State->dX1[ 3];	// ATT 0
		temp[ 4] = State->X[ 4] + (0.5) * dt * State->dX1[ 4];	// ATT 1
		temp[ 5] = State->X[ 5] + (0.5) * dt * State->dX1[ 5];	// ATT 2
		temp[ 6] = State->X[ 6] + (0.5) * dt * State->dX1[ 6];	// ATT 3

		temp[ 7] = State->X[ 7] + (0.5) * dt * State->dX1[ 7];	// VEL X
		temp[ 8] = State->X[ 8] + (0.5) * dt * State->dX1[ 8];	// VEL Y
		temp[ 9] = State->X[ 9] + (0.5) * dt * State->dX1[ 9];	// VEL Z

		temp[10] = State->X[10] + (0.5) * dt * State->dX1[10];	// ANV p
		temp[11] = State->X[11] + (0.5) * dt * State->dX1[11];	// ANV Q
		temp[12] = State->X[12] + (0.5) * dt * State->dX1[12];	// ANV R

		dynamic( State->dX2, temp, State->deuler, Rocket, ForceMoment );

	case 3:
		temp[ 0] = State->X[ 0] + (0.5) * dt * State->dX2[ 0];	// POS N
		temp[ 1] = State->X[ 1] + (0.5) * dt * State->dX2[ 1];	// POS E
		temp[ 2] = State->X[ 2] + (0.5) * dt * State->dX2[ 2];	// POS D

		temp[ 3] = State->X[ 3] + (0.5) * dt * State->dX2[ 3];	// ATT 0
		temp[ 4] = State->X[ 4] + (0.5) * dt * State->dX2[ 4];	// ATT 1
		temp[ 5] = State->X[ 5] + (0.5) * dt * State->dX2[ 5];	// ATT 2
		temp[ 6] = State->X[ 6] + (0.5) * dt * State->dX2[ 6];	// ATT 3

		temp[ 7] = State->X[ 7] + (0.5) * dt * State->dX2[ 7];	// VEL X
		temp[ 8] = State->X[ 8] + (0.5) * dt * State->dX2[ 8];	// VEL Y
		temp[ 9] = State->X[ 9] + (0.5) * dt * State->dX2[ 9];	// VEL Z

		temp[10] = State->X[10] + (0.5) * dt * State->dX2[10];	// ANV p
		temp[11] = State->X[11] + (0.5) * dt * State->dX2[11];	// ANV Q
		temp[12] = State->X[12] + (0.5) * dt * State->dX2[12];	// ANV R

		dynamic( State->dX3, temp, State->deuler, Rocket, ForceMoment );

	case 4:
		temp[ 0] = State->X[ 0] + dt * State->dX3[ 0];	// POS N
		temp[ 1] = State->X[ 1] + dt * State->dX3[ 1];	// POS E
		temp[ 2] = State->X[ 2] + dt * State->dX3[ 2];	// POS D

		temp[ 3] = State->X[ 3] + dt * State->dX3[ 3];	// ATT 0
		temp[ 4] = State->X[ 4] + dt * State->dX3[ 4];	// ATT 1
		temp[ 5] = State->X[ 5] + dt * State->dX3[ 5];	// ATT 2
		temp[ 6] = State->X[ 6] + dt * State->dX3[ 6];	// ATT 3

		temp[ 7] = State->X[ 7] + dt * State->dX3[ 7];	// VEL X
		temp[ 8] = State->X[ 8] + dt * State->dX3[ 8];	// VEL Y
		temp[ 9] = State->X[ 9] + dt * State->dX3[ 9];	// VEL Z

		temp[10] = State->X[10] + dt * State->dX3[10];	// ANV p
		temp[11] = State->X[11] + dt * State->dX3[11];	// ANV Q
		temp[12] = State->X[12] + dt * State->dX3[12];	// ANV R

		dynamic( State->dX4, temp, State->deuler, Rocket, ForceMoment );
	}
}


void UpdateState( st_State *State, st_Rocket *Rocket )
{
	double dt = State->dt;

	Rocket->POS[0] = State->X[ 0] + one_of_six * dt * ( State->dX1[ 0] + 2 * State->dX2[ 0] + 2 * State->dX3[ 0] + State->dX4[ 0] );
	Rocket->POS[1] = State->X[ 1] + one_of_six * dt * ( State->dX1[ 1] + 2 * State->dX2[ 1] + 2 * State->dX3[ 1] + State->dX4[ 1] );
	Rocket->POS[2] = State->X[ 2] + one_of_six * dt * ( State->dX1[ 2] + 2 * State->dX2[ 2] + 2 * State->dX3[ 2] + State->dX4[ 2] );

	Rocket->ATT[0] = State->X[ 3] + one_of_six * dt * ( State->dX1[ 3] + 2 * State->dX2[ 3] + 2 * State->dX3[ 3] + State->dX4[ 3] );
	Rocket->ATT[1] = State->X[ 4] + one_of_six * dt * ( State->dX1[ 4] + 2 * State->dX2[ 4] + 2 * State->dX3[ 4] + State->dX4[ 4] );
	Rocket->ATT[2] = State->X[ 5] + one_of_six * dt * ( State->dX1[ 5] + 2 * State->dX2[ 5] + 2 * State->dX3[ 5] + State->dX4[ 5] );

	Rocket->V_B[0] = State->X[ 7] + one_of_six * dt * ( State->dX1[ 7] + 2 * State->dX2[ 7] + 2 * State->dX3[ 7] + State->dX4[ 7] );
	Rocket->V_B[1] = State->X[ 8] + one_of_six * dt * ( State->dX1[ 8] + 2 * State->dX2[ 8] + 2 * State->dX3[ 8] + State->dX4[ 8] );
	Rocket->V_B[2] = State->X[ 9] + one_of_six * dt * ( State->dX1[ 9] + 2 * State->dX2[ 9] + 2 * State->dX3[ 9] + State->dX4[ 9] );

	Rocket->ANV[0] = State->X[10] + one_of_six * dt * ( State->dX1[10] + 2 * State->dX2[10] + 2 * State->dX3[10] + State->dX4[10] );
	Rocket->ANV[1] = State->X[11] + one_of_six * dt * ( State->dX1[11] + 2 * State->dX2[11] + 2 * State->dX3[11] + State->dX4[11] );
	Rocket->ANV[2] = State->X[12] + one_of_six * dt * ( State->dX1[12] + 2 * State->dX2[12] + 2 * State->dX3[12] + State->dX4[12] );

	// DCM
	dcm_from_euler( Rocket->DCM, Rocket->ATT );

	// Velocity
	Rocket->VEL[0] = Rocket->DCM[0][0] * Rocket->V_B[0] + Rocket->DCM[1][0] * Rocket->V_B[1] + Rocket->DCM[2][0] * Rocket->V_B[2];
	Rocket->VEL[1] = Rocket->DCM[0][1] * Rocket->V_B[0] + Rocket->DCM[1][1] * Rocket->V_B[1] + Rocket->DCM[2][1] * Rocket->V_B[2];
	Rocket->VEL[2] = Rocket->DCM[0][2] * Rocket->V_B[0] + Rocket->DCM[1][2] * Rocket->V_B[1] + Rocket->DCM[2][2] * Rocket->V_B[2];

	State->t += dt;
}


// Legacy

/* for <<<<<<<<<<<<<<<<<<<< Uploadstate >>>>>>>>>>>>>>>>>>>> */
//	State->X[3] = Rocket->QUT[0];
//	State->X[4] = Rocket->QUT[1];
//	State->X[5] = Rocket->QUT[2];
//	State->X[6] = Rocket->QUT[3];

/* for <<<<<<<<<<<<<<<<<<<< dynamic >>>>>>>>>>>>>>>>>>>> */
//	dX[ 3] = (0.5) * (  X[10] * X[6] + X[11] * X[5] - X[12] * X[4] );
//	dX[ 4] = (0.5) * ( -X[10] * X[5] + X[11] * X[6] + X[12] * X[3] );
//	dX[ 5] = (0.5) * (  X[10] * X[4] - X[11] * X[3] + X[12] * X[6] );
//	dX[ 6] = (0.5) * ( -X[10] * X[3] - X[11] * X[4] - X[12] * X[5] );

/* for <<<<<<<<<<<<<<<<<<<< Updatestate >>>>>>>>>>>>>>>>>>>> */
//	double qut[4];
//	double invqutmng;

//	qut[0] = State->X[ 3] + one_of_six * dt * ( State->dX1[ 3] + 2 * State->dX2[ 3] + 2 * State->dX3[ 3] + State->dX4[ 3] );
//	qut[1] = State->X[ 4] + one_of_six * dt * ( State->dX1[ 4] + 2 * State->dX2[ 4] + 2 * State->dX3[ 4] + State->dX4[ 4] );
//	qut[2] = State->X[ 5] + one_of_six * dt * ( State->dX1[ 5] + 2 * State->dX2[ 5] + 2 * State->dX3[ 5] + State->dX4[ 5] );
//	qut[3] = State->X[ 6] + one_of_six * dt * ( State->dX1[ 6] + 2 * State->dX2[ 6] + 2 * State->dX3[ 6] + State->dX4[ 6] );
//
//	invqutmng = 1. / sqrt( qut[0]*qut[0] + qut[1]*qut[1] + qut[2]*qut[2] + qut[3]*qut[3] );
//
//	Rocket->QUT[0] = qut[0] * invqutmng;
//	Rocket->QUT[1] = qut[1] * invqutmng;
//	Rocket->QUT[2] = qut[2] * invqutmng;
//	Rocket->QUT[3] = qut[3] * invqutmng;

//	dcm_from_quatr( Rocket->DCM, Rocket->QUT );

//	Rocket->ATT[0] = atan2(  Rocket->DCM[1][2], Rocket->DCM[2][2] );
//	Rocket->ATT[1] = atan2( -Rocket->DCM[0][2], sqrt( Rocket->DCM[0][0] * Rocket->DCM[0][0] + Rocket->DCM[0][1] * Rocket->DCM[0][1] ) );
//	Rocket->ATT[2] = atan2(  Rocket->DCM[0][1], Rocket->DCM[0][0] );
