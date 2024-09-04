/*
 * control.h
 *
 *  Created on: Jun 11, 2024
 *      Author: daegeun
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdint.h>
#include <math.h>

#include <constants.h>

#include <rocket.h>


typedef struct {
	double TVC_w;
	double TVC_j;

	double RCS_w;
	double RCS_j;

	double TVC_Kp;
	double TVC_Kd;

	double RCS_Kp;
	double RCS_Kd;

	double theta[2];

	double max_theta;

	double Thrust;

	/* Actuator Input Start */
	uint8_t  CUTOFF;
	uint16_t PWM;

	uint16_t RCS1;
	uint16_t RCS2;
	uint16_t RCS3;
	uint16_t RCS4;

	uint16_t TVC_1;
	uint16_t TVC_2;
	/* Actuator Input End */
} st_Controller;


extern st_Controller Controller;

void Init_Controller( st_Controller *Controller, const double Mass );

void Do_Control( st_Controller *Controller, const st_Rocket *Rocket, const double *Command );

#endif /* INC_CONTROL_H_ */
