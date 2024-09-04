/*
 * dynamic.h
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#ifndef INC_DYNAMIC_H_
#define INC_DYNAMIC_H_

#include <rocket.h>
#include <force_moment.h>
#include <dcm.h>


#define NumberOfState ( 13 )

typedef struct {
	double X[NumberOfState];

	double dX1[NumberOfState];
	double dX2[NumberOfState];
	double dX3[NumberOfState];
	double dX4[NumberOfState];

	double deuler[3][3];

	double  t;
	double dt;
} st_State;


extern st_State State;


void Init_State( st_State *State );

void UploadState( st_State *State, const st_Rocket *Rocket );
void UpdateState( st_State *State,       st_Rocket *Rocket );

void dynamic( double *dX, const double *X, const double deuler[][3], const st_Rocket *Rocket, const st_ForceMoment *ForceMoment );

void dxdt( st_State *State, const st_Rocket *Rocket, const st_ForceMoment *ForceMoment, const uint8_t Order );

#endif /* INC_DYNAMIC_H_ */
