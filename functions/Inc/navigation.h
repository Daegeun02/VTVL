/*
 * navigation.h
 *
 *  Created on: Jun 19, 2024
 *      Author: daegeun
 */

#ifndef INC_NAVIGATION_H_
#define INC_NAVIGATION_H_

#include <math.h>

#include <constants.h>

#include <dcm.h>


void Do_NED_to_LLH( double *LLH, const double *NED, const double *LL0 );

void Do_LLH_to_ECEF( double *ECEF, const double *LLH );

void Do_LLH_to_NED( double *NED, const double *LLH, const double *LL0 );

double _abs( double value );

#endif /* INC_NAVIGATION_H_ */
