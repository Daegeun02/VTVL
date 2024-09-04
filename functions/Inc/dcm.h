/*
 * dcm.h
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#ifndef INC_DCM_H_
#define INC_DCM_H_

#include <math.h>



void dcm_from_euler( double DCM[][3], const double *euler );

void dcm_from_quatr( double DCM[][3], const double *quatr );

void euler_to_quatr( double *quatr, const double *euler );

#endif /* INC_DCM_H_ */
