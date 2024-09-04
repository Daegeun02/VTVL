/*
 * guidance.h
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#ifndef INC_GUIDANCE_H_
#define INC_GUIDANCE_H_

#include <stdint.h>
#include <math.h>

#include "rocket.h"
#include "actuator.h"


#define TakeOff_Height ( 2 )


typedef enum {
	Guidance_TakeOff = 40,
	Guidance_Landing = 41,
	Guidance_DoHover = 42,
	Guidance_HorMove = 43,
	Guidance_VerMove = 44,
	Guidance_Suspend = 49,
	Guidance_END     = 99,
} GuidancePhase;


typedef struct {
	double TargetPosition[3];
	double PhasingPosition[3];

	double AccCommand[3];

	double Command[4];

	double w_v;
	double j_v;
	double w_h;
	double j_h;

	double Kp_v;
	double Kd_v;
	double Kp_h;
	double Kd_h;

	double max_ATT;

	double Target_N;
	double Target_E;
	double Target_D;

	double PhasingTime;
	double t;
	double T;

	double Hz;
	double dt;

	uint8_t GuidancePhase;
} st_Guidance;


extern st_Guidance Guidance;

void Init_Guidance( st_Guidance *Guidance );

void Do_Guidance( st_Guidance *Guidance, const st_Rocket *Rocket );

#endif /* INC_GUIDANCE_H_ */
