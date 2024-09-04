/*
 * force_moment.h
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#ifndef INC_FORCE_MOMENT_H_
#define INC_FORCE_MOMENT_H_

#include <actuator.h>
#include <rocket.h>



typedef struct {
	double ForceA[3];
	double ForceB[3];
	double Moment[3];
} st_ForceMoment;

extern st_ForceMoment ForceMoment;


void Cal_ForceMoment( st_ForceMoment *ForceMoment, const st_Actuator *Actuator, const st_Rocket *Rocket );

#endif /* INC_FORCE_MOMENT_H_ */
